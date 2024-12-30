//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers esp-hal/unstable

#![no_std]
#![no_main]
use alloc::{boxed::Box, sync::Arc};
use embassy_executor::Spawner;
use embassy_futures::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_storage::{ReadStorage, Storage};
use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_hal::{
    gpio::{AnyPin, Input, Level, Output, Pull},
    i2c::master::{Config, I2c},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_storage::FlashStorage;
use log::info;
use mpu6050::{device::GyroRange, Mpu6050};
use postcard::from_bytes;
use serde::{Deserialize, Serialize};

extern crate alloc;
use core::mem::MaybeUninit;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

type CalibratingSignal = Arc<Signal<CriticalSectionRawMutex, ()>>;
type CalibratingBool = Arc<Mutex<CriticalSectionRawMutex, bool>>;

#[derive(Debug, Serialize, Deserialize)]
struct FlashData {
    gyro_offset: [f32; 3],
}

#[main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!("Initialising...");

    println!("Peripherals...");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Heap...");
    init_heap();

    println!("Embassy...");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    println!("GPIO...");
    let mut led = Output::new(AnyPin::from(peripherals.GPIO25), Level::Low);
    let mut vibration_motor = Output::new(AnyPin::from(peripherals.GPIO26), Level::Low);
    let calibration_button = Input::new(AnyPin::from(peripherals.GPIO27), Pull::Up);

    println!("Flash...");
    // Initialize the ESP32 flash memory to store the calibration data
    let mut flash = FlashStorage::new();

    let flash_addr = 0x9000;
    println!("Flash size = {}", flash.capacity());
    let mut data = FlashData {
        gyro_offset: [0.0, 0.0, 0.0],
    };

    // Read the calibration data from flash
    let mut buffer = [0u8; core::mem::size_of::<FlashData>()];
    if let Ok(()) = flash.read(flash_addr, &mut buffer) {
        data = from_bytes(&buffer).unwrap();
        println!("Read flash data: {:?}", data.gyro_offset);
    }

    // Initialize the gyroscope (MPU6050)
    let gyroscope = I2c::new(peripherals.I2C0, Config::default());

    let gyro_interrupt = Input::new(AnyPin::from(peripherals.GPIO3), Pull::Up);

    let mut delay = Delay;
    let mut mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>> =
        Arc::new(Mutex::new(Mpu6050::new(gyroscope)));

    {
        let mut mpu = mpu.lock().await;
        mpu.init(&mut delay).expect("MPU6050 init failed");

        mpu.set_gyro_range(GyroRange::D250)
            .expect("set_gyro_range failed");
    }
    // Vibrate twice to tell the user that the device is ready
    led.set_high();
    vibration_motor.set_high();
    Timer::after(Duration::from_millis(100)).await;
    led.set_low();
    vibration_motor.set_low();

    Timer::after(Duration::from_millis(100)).await;

    led.set_high();
    vibration_motor.set_high();
    Timer::after(Duration::from_millis(100)).await;
    vibration_motor.set_low();
    led.set_low();

    let calibrate_signal: CalibratingSignal =
        Arc::new(Signal::<CriticalSectionRawMutex, ()>::new());

    let calibrating: CalibratingBool = Arc::new(Mutex::new(false));

    // Start the calibration task
    _spawner
        .spawn(calibrate(
            Box::new(calibration_button),
            calibrate_signal.clone(),
            calibrating.clone(),
            mpu.clone(),
        ))
        .unwrap();

    _spawner
        .spawn(check_orientation_and_vibrate(
            mpu.clone(),
            Box::new(gyro_interrupt),
            Box::new(vibration_motor),
            calibrate_signal.clone(),
            calibrating.clone(),
        ))
        .unwrap();
}

#[embassy_executor::task]
async fn calibrate(
    mut calibration_button: Box<Input<'static>>,
    calibrating_finished: CalibratingSignal,
    calibrating: CalibratingBool,
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
) {
    // Wait for the calibration button to be pressed
    calibration_button.wait_for_high().await;

    {
        let mut calibrating = calibrating.lock().await;
        *calibrating = true;
    }

    // Wait for the calibration button to be released
    calibration_button.wait_for_low().await;

    // Wait for 1 second to allow the user to move their hands away from the device
    Timer::after(Duration::from_secs(1)).await;

    // Calibrate the gyroscope, assuming that the device is stationary and lying flat on its back
    {
        let mut mpu = mpu.lock().await;
        mpu.get_acc_angles().unwrap();
    }

    // Signal that the calibration is finished
    {
        let mut calibrating = calibrating.lock().await;
        *calibrating = false;
    }
    calibrating_finished.signal(());
}

#[embassy_executor::task]
async fn check_orientation_and_vibrate(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
    mut gyro_interrupt: Box<Input<'static>>,
    mut vibration_motor: Box<Output<'static>>,
    calibrating_finished: CalibratingSignal,
    calibrating: CalibratingBool,
) {
    // If the device is upside down and has been for 5 seconds, vibrate in increasing intervals and intensities
    // Otherwise the device is not upside down, turn off the vibration motor
    let mut upside_down_since: Option<Instant> = None;

    // Setup motion detection
    {
        let mut mpu = mpu.lock().await;
        mpu.setup_motion_detection().unwrap();
    }

    loop {
        let is_calibrating = { *(calibrating.lock().await) };
        if is_calibrating {
            // Wait for the calibration to finish
            calibrating_finished.wait().await;
        }

        let mut mpu = mpu.lock().await;

        // Read the gyroscope data
        let gyro = mpu.get_gyro().unwrap();

        // Determine if the device is upside down
        let is_upside_down = gyro[2] - 1.0 < 0.0;

        if is_upside_down {
            if upside_down_since.is_none() {
                upside_down_since = Some(Instant::now());
            }
        } else {
            upside_down_since = None;
        }

        // Vibrate if the device has been upside down for a while
        if let Some(since) = upside_down_since {
            if Instant::now() - since > Duration::from_secs(5) {
                vibration_motor.set_high();
                Timer::after(Duration::from_millis(100)).await;
                vibration_motor.set_low();
                upside_down_since = None;
            }
        }

        // Wait for the next interrupt, or 5s, whichever comes first
        let sig = gyro_interrupt.wait_for_low();

        let timer = Timer::after(Duration::from_millis(5000));

        let res = select::select(sig, timer).await;

        match res {
            select::Either::First(_) => {
                // Gyro interrupt
                info!("Gyro interrupt");
            }
            select::Either::Second(_) => {
                // Timeout
                info!("Timeout");
            }
        }
    }
}
