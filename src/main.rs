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
use embassy_futures::select::select;
use embassy_sync::pubsub::{PubSubBehavior, PubSubChannel};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_hal::rtc_cntl::sleep::RtcSleepConfig;
use esp_hal::{
    gpio::{Input, Level, Output, Pull},
    i2c::master::{Config, I2c},
    timer::timg::TimerGroup,
};
use log::{debug, info};
use mpu6050::Mpu6050;

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

type PowerSignal = PubSubChannel<CriticalSectionRawMutex, (), 1, 2, 1>;
type PowerValue = Mutex<CriticalSectionRawMutex, bool>;

static POWER_SIGNAL: PowerSignal = (PubSubChannel::<CriticalSectionRawMutex, (), 1, 2, 1>::new());
static POWER_VALUE: PowerValue = Mutex::new(true);
static MPU_READY: PowerSignal = (PubSubChannel::<CriticalSectionRawMutex, (), 1, 2, 1>::new());

#[main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    info!("Initialising...");
    info!("Heap...");
    init_heap();

    info!("Peripherals...");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Embassy...");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    info!("GPIO...");
    let mut mpu_power = Output::new(peripherals.GPIO23, Level::High);
    let led: Arc<Mutex<CriticalSectionRawMutex, Output>> =
        Arc::new(Mutex::new(Output::new(peripherals.GPIO22, Level::Low)));
    let vibration_motor = Arc::new(Mutex::new(Output::new(peripherals.GPIO26, Level::Low)));
    let power_button = Input::new(peripherals.GPIO5, Pull::None);

    // Wait for the MPU6050 to power up
    Timer::after(Duration::from_millis(100)).await;

    // Initialize the accelerometer (MPU6050)
    let accelerometer = I2c::new(
        peripherals.I2C0,
        Config {
            frequency: fugit::Rate::<u32, 1, 1>::kHz(100),
            ..Default::default()
        },
    )
    .with_sda(peripherals.GPIO33)
    .with_scl(peripherals.GPIO32);

    let mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>> =
        Arc::new(Mutex::new(Mpu6050::new(accelerometer)));

    // Start the calibration task
    _spawner.spawn(handle_power_button(power_button)).unwrap();

    _spawner
        .spawn(check_orientation_and_vibrate(
            mpu.clone(),
            vibration_motor.clone(),
        ))
        .unwrap();

    let mut fef = RtcSleepConfig::default();
    let led_clone = led.clone();
    let vib = vibration_motor.clone();
    let mut sub = POWER_SIGNAL.subscriber().unwrap();
    loop {
        let power = { *POWER_VALUE.lock().await };

        if power {
            info!("Waking up");
            fef.set_deep_slp(false);
            info!("Setting MPU power high");
            mpu_power.set_high();
            info!("Waiting for MPU to power up");
            Timer::after(Duration::from_millis(100)).await;

            info!("Initialising MPU");
            let delay = Delay;
            init_mpu(mpu.clone(), delay).await;

            info!("Device ready");

            // Vibrate twice to tell the user that the device is ready
            {
                let mut led = led_clone.lock().await;
                let mut vibration_motor = vib.lock().await;
                led.set_low();
                vibration_motor.set_high();
            }
            Timer::after(Duration::from_millis(100)).await;
            {
                let mut led = led_clone.lock().await;
                let mut vibration_motor = vib.lock().await;
                led.set_high();
                vibration_motor.set_low();
            }

            Timer::after(Duration::from_millis(100)).await;

            {
                let mut led = led_clone.lock().await;
                let mut vibration_motor = vib.lock().await;
                led.set_low();
                vibration_motor.set_high();
            }
            Timer::after(Duration::from_millis(100)).await;
            {
                let mut led = led_clone.lock().await;
                let mut vibration_motor = vib.lock().await;
                vibration_motor.set_low();
                led.set_high();
            }
        } else {
            info!("Going to sleep");
            // Set ESP32 to low power mode
            mpu_power.set_low();
            {
                let mut led = led_clone.lock().await;
                let mut vibration_motor = vib.lock().await;
                vibration_motor.set_low();
                led.set_high();
            }
            fef.set_deep_slp(true);
        }

        info!("Waiting for power signal");

        sub.next_message().await;

        info!("Power signal received");
    }
}

async fn init_mpu(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>>,
    mut delay: Delay,
) {
    let mut mpu = mpu.lock().await;
    mpu.init(&mut delay).expect("MPU6050 init failed");

    // mpu.set_accel_range(mpu6050::device::AccelRange::G16)
    //     .expect("set_acc_range failed");

    mpu.set_temp_enabled(true).expect("set_temp_enabled failed");

    // mpu.set_clock_source(mpu6050::device::CLKSEL::EXT_32p7)
    //     .expect("set_clock_source failed");

    // mpu.set_gyro_range(GyroRange::D250)
    //     .expect("set_gyro_range failed");

    info!("MPU6050 initialized");

    MPU_READY.publish_immediate(());
}

#[embassy_executor::task]
async fn handle_power_button(mut power_button: Input<'static>) {
    loop {
        // Wait for the power button to be pressed
        power_button.wait_for_rising_edge().await;

        info!("Power button pressed");
        {
            let mut val = POWER_VALUE.lock().await;
            *val = !*val;
        }
        POWER_SIGNAL.publish_immediate(());
        power_button.wait_for_falling_edge().await;

        info!("Power button released");

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn check_orientation_and_vibrate(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
    vibration_motor: Arc<Mutex<CriticalSectionRawMutex, Output<'static>>>,
) {
    // If the device is upside down and has been for 5 seconds, vibrate in increasing intervals and intensities
    // Otherwise the device is not upside down, turn off the vibration motor
    let mut upside_down_since: Option<Instant> = None;

    let mut mpu_ready = MPU_READY.subscriber().unwrap();
    let mut power_signal = POWER_SIGNAL.subscriber().unwrap();

    loop {
        {
            info!("Getting power value");
            let val = { *POWER_VALUE.lock().await };
            if !val {
                info!("Power off");

                loop {
                    let power = { *POWER_VALUE.lock().await };
                    if power {
                        mpu_ready.next_message().await;
                        info!("MPU ready");
                        break;
                    }
                    power_signal.next_message().await;
                }
                info!("Power on");
            }
        }

        let acc = {
            let mut mpu = mpu.lock().await;
            // Read the accelerometer data
            mpu.get_acc().unwrap_or([0.0, 0.0, 0.0].into())
        };

        // Determine if the device is upside down
        let z_acc = acc[2];
        let is_upside_down = z_acc < -0.6;

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
                {
                    let mut vibration_motor = vibration_motor.lock().await;
                    vibration_motor.set_high();
                }
                Timer::after(Duration::from_millis(1000)).await;
                {
                    let mut vibration_motor = vibration_motor.lock().await;
                    vibration_motor.set_low();
                }
                upside_down_since = None;
            }
        }

        let power_sig = power_signal.next_message();
        let timeout = Timer::after(Duration::from_millis(5000));

        select(power_sig, timeout).await;
    }
}
