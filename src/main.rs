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
    gpio::{Input, Level, Output, Pull},
    i2c::master::{Config, I2c},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_storage::FlashStorage;
use log::info;
use mpu6050::Mpu6050;
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

type CalibratingSignal = Arc<Signal<CriticalSectionRawMutex, FlashData>>;
type CalibratingBool = Arc<Mutex<CriticalSectionRawMutex, bool>>;

#[derive(Debug, Serialize, Deserialize, Clone)]
struct FlashData {
    acc_offset: [f32; 3],
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
    let mut led = Output::new(peripherals.GPIO0, Level::Low);
    let mut vibration_motor = Output::new(peripherals.GPIO26, Level::Low);
    let calibration_button = Input::new(peripherals.GPIO27, Pull::Up);

    println!("Flash...");
    // Initialize the ESP32 flash memory to store the calibration data
    let mut flash = FlashStorage::new();

    let flash_addr = 0x9000;
    println!("Flash size = {}", flash.capacity());
    let mut data = FlashData {
        acc_offset: [0.0, 0.0, 0.0],
    };

    // Read the calibration data from flash
    let mut buffer = [0u8; core::mem::size_of::<FlashData>()];
    if let Ok(()) = flash.read(flash_addr, &mut buffer) {
        data = from_bytes(&buffer).unwrap();
    }

    println!("Read flash data: {:?}", data.acc_offset);

    // Initialize the accscope (MPU6050)
    let accscope = I2c::new(
        peripherals.I2C0,
        Config {
            frequency: fugit::Rate::<u32, 1, 1>::kHz(100),
            ..Default::default()
        },
    )
    .with_sda(peripherals.GPIO33)
    .with_scl(peripherals.GPIO32);

    let acc_interrupt = Input::new((peripherals.GPIO25), Pull::Up);

    let mut delay = Delay;
    let mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>> =
        Arc::new(Mutex::new(Mpu6050::new(accscope)));

    {
        let mut mpu = mpu.lock().await;
        mpu.init(&mut delay).expect("MPU6050 init failed");

        mpu.set_accel_range(mpu6050::device::AccelRange::G16)
            .expect("set_acc_range failed");
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
        Arc::new(Signal::<CriticalSectionRawMutex, FlashData>::new());

    let calibrating: CalibratingBool = Arc::new(Mutex::new(false));

    // Start the calibration task
    _spawner
        .spawn(calibrate(
            Box::new(calibration_button),
            calibrate_signal.clone(),
            calibrating.clone(),
            mpu.clone(),
            flash,
            data.clone(),
        ))
        .unwrap();

    _spawner
        .spawn(check_orientation_and_vibrate(
            mpu.clone(),
            Box::new(acc_interrupt),
            Box::new(vibration_motor),
            calibrate_signal.clone(),
            calibrating.clone(),
            data,
        ))
        .unwrap();
}

#[embassy_executor::task]
async fn calibrate(
    mut calibration_button: Box<Input<'static>>,
    calibrating_finished: CalibratingSignal,
    calibrating: CalibratingBool,
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
    mut flash: FlashStorage,
    mut data: FlashData,
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

    // Calibrate the accscope, assuming that the device is stationary and lying flat on its back
    {
        let mut mpu = mpu.lock().await;
        let mut acc_sum = [0.0, 0.0, 0.0];

        for _ in 0..100 {
            let acc = mpu.get_acc().unwrap();
            acc_sum[0] += acc[0];
            acc_sum[1] += acc[1];
            acc_sum[2] += acc[2];

            Timer::after(Duration::from_millis(100)).await;
        }

        data.acc_offset[0] = acc_sum[0] / 100.0;
        data.acc_offset[1] = acc_sum[1] / 100.0;
        data.acc_offset[2] = acc_sum[2] / 100.0;

        // Write the calibration data to flash
        let buffer = postcard::to_allocvec(&data).unwrap();
        flash.write(0x9000, &buffer).unwrap();

        info!("Calibration data: {:?}", data.acc_offset);
    }

    // Signal that the calibration is finished
    {
        let mut calibrating = calibrating.lock().await;
        *calibrating = false;
    }
    calibrating_finished.signal(data);
}

#[embassy_executor::task]
async fn check_orientation_and_vibrate(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
    mut acc_interrupt: Box<Input<'static>>,
    mut vibration_motor: Box<Output<'static>>,
    calibrating_finished: CalibratingSignal,
    calibrating: CalibratingBool,
    data: FlashData,
) {
    // If the device is upside down and has been for 5 seconds, vibrate in increasing intervals and intensities
    // Otherwise the device is not upside down, turn off the vibration motor
    let mut upside_down_since: Option<Instant> = None;

    let mut data = data.clone();

    // Setup motion detection
    {
        let mut mpu = mpu.lock().await;
        mpu.setup_motion_detection().unwrap();
    }

    loop {
        let is_calibrating = { *(calibrating.lock().await) };
        if is_calibrating {
            // Wait for the calibration to finish
            data = calibrating_finished.wait().await;
        }

        let mut mpu = mpu.lock().await;

        // Read the accscope data
        let acc = mpu.get_acc().unwrap();

        // Determine if the device is upside down
        let z_acc = acc[2];
        let is_upside_down = z_acc < -5.0;

        info!("Acc: {:?}", z_acc);
        if is_upside_down {
            info!("Upside down");
            if upside_down_since.is_none() {
                upside_down_since = Some(Instant::now());
            }
        } else {
            info!("Not upside down");
            upside_down_since = None;
        }

        // Vibrate if the device has been upside down for a while
        if let Some(since) = upside_down_since {
            if Instant::now() - since > Duration::from_secs(5) {
                info!("Upside down for 5 seconds, vibrating");
                vibration_motor.set_high();
                Timer::after(Duration::from_millis(1000)).await;
                vibration_motor.set_low();
                upside_down_since = None;
            }
        }

        // Wait for the next interrupt, or 5s, whichever comes first
        let sig = acc_interrupt.wait_for_low();

        let timer = Timer::after(Duration::from_millis(5000));

        let res = select::select(sig, timer).await;

        match res {
            select::Either::First(_) => {
                // Acc interrupt
                info!("Acc interrupt");
            }
            select::Either::Second(_) => {
                // Timeout
                info!("Timeout");
            }
        }
    }
}
