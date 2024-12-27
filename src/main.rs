//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers esp-hal/unstable

#![no_std]
#![no_main]

use alloc::boxed::Box;
use embassy_executor::{task, Spawner};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{AnyPin, Input, Level, Output, OutputPin, Pull},
    i2c::master::{AnyI2c, Config, I2c},
    interrupt,
    macros::handler,
    timer::timg::TimerGroup,
    xtensa_lx::interrupt,
};

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

#[task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

static gyroscope_signal: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// static gyroscope_pin: Mutex<CriticalSectionRawMutex, Option<AnyI2c>> = Mutex::new(None);

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();

    esp_println::println!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut led = Output::new(AnyPin::from(peripherals.GPIO0), Level::Low);
    let mut vibration_motor = Output::new(AnyPin::from(peripherals.GPIO1), Level::Low);

    // Initialize the gyroscope (MPU6050)
    let mut gyroscope = I2c::new(peripherals.I2C0, Config::default()).into_async();
    let mut gyroscope_interrupt = Input::new(AnyPin::from(peripherals.GPIO2), Pull::Up);

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

    // gyroscope_pin.lock().await.replace(gyroscope);

    loop {
        // Get orientation data from the gyroscope
        gyroscope_interrupt.wait_for_rising_edge().await;

        // Read the gyroscope data
        let mut data = [0; 14];
        gyroscope
            .write_read(0x68, &[0x3B], &mut data)
            .await
            .unwrap();

        let acc_x = ((data[0] as i16) << 8) | data[1] as i16;
        let acc_y = ((data[2] as i16) << 8) | data[3] as i16;
        let acc_z = ((data[4] as i16) << 8) | data[5] as i16;

        let temp = ((data[6] as i16) << 8) | data[7] as i16;

        let gyro_x = ((data[8] as i16) << 8) | data[9] as i16;
        let gyro_y = ((data[10] as i16) << 8) | data[11] as i16;
        let gyro_z = ((data[12] as i16) << 8) | data[13] as i16;

        // Determine if the device is upside down
        let is_upside_down = gyro_z < -1000;

        // Vibrate if the device is upside down
        if is_upside_down {
            vibration_motor.set_high();
            Timer::after(Duration::from_millis(100)).await;
            vibration_motor.set_low();
        }

        Timer::after(Duration::from_millis(5_000)).await;
    }
}

// Gyroscope interrupt handler
#[handler]
fn GPIO2() {
    esp_println::println!("Gyroscope interrupt!");
    gyroscope_signal.signal(());
}
