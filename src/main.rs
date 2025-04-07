//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers esp-hal/unstable

#![no_std]
#![no_main]
use alloc::sync::Arc;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::gpio::OutputConfig;
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::{
    gpio::{Level, Output},
    i2c::master::{Config, I2c},
    timer::timg::TimerGroup,
};
use esp_hal_embassy::main;
use log::info;
use mpu6050::Mpu6050;

extern crate alloc;
use core::mem::MaybeUninit;

const SLEEP_DURATION_SECS: u8 = 5;
const UPSIDE_DOWN_THRESHOLD_SECS: u8 = 5;

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
    Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // MPU6050 GND
    let mut mpu_power = Output::new(peripherals.GPIO26, Level::High, OutputConfig::default()); // MPU6050 VCC
    let led: Arc<Mutex<CriticalSectionRawMutex, Output>> = Arc::new(Mutex::new(Output::new(
        peripherals.GPIO12,
        Level::Low,
        OutputConfig::default(),
    )));
    let vibration_motor: Arc<Mutex<CriticalSectionRawMutex, Output<'_>>> = Arc::new(Mutex::new(
        Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default()),
    ));

    // Wait for the MPU6050 to power up
    Timer::after(Duration::from_millis(100)).await;

    // Initialize the accelerometer (MPU6050)
    let accelerometer = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO32)
    .with_scl(peripherals.GPIO33);

    let mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>> =
        Arc::new(Mutex::new(Mpu6050::new(accelerometer)));

    let rtc = Rtc::new(peripherals.LPWR);

    let rtc_mutex = Arc::new(Mutex::new(rtc));

    _spawner
        .spawn(check_orientation_and_vibrate(
            mpu.clone(),
            vibration_motor.clone(),
            rtc_mutex.clone(),
        ))
        .unwrap();

    info!("Setting MPU power high");
    mpu_power.set_high();
    info!("Waiting for MPU to power up");
    Timer::after(Duration::from_millis(100)).await;

    info!("Initialising MPU");
    let delay = Delay;
    init_mpu(mpu.clone(), delay).await;

    info!("Device ready");

    for _ in 0..2 {
        vibrate(
            Duration::from_millis(100),
            led.clone(),
            vibration_motor.clone(),
        )
        .await;

        Timer::after(Duration::from_millis(100)).await;
    }
}

async fn vibrate(
    duration: Duration,
    led_clone: Arc<Mutex<CriticalSectionRawMutex, Output<'static>>>,
    vib: Arc<Mutex<CriticalSectionRawMutex, Output<'static>>>,
) {
    activate_vibration(led_clone.clone(), vib.clone()).await;
    Timer::after(duration).await;
    deactivate_vibration(led_clone.clone(), vib.clone()).await;
}

async fn deactivate_vibration(
    led_clone: Arc<Mutex<CriticalSectionRawMutex, Output<'_>>>,
    vib: Arc<Mutex<CriticalSectionRawMutex, Output<'_>>>,
) {
    led_clone.lock().await.set_high();
    vib.lock().await.set_low();
}

async fn activate_vibration(
    led_clone: Arc<Mutex<CriticalSectionRawMutex, Output<'_>>>,
    vib: Arc<Mutex<CriticalSectionRawMutex, Output<'_>>>,
) {
    led_clone.lock().await.set_low();
    vib.lock().await.set_high();
}

async fn init_mpu(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'_, esp_hal::Blocking>>>>,
    mut delay: Delay,
) {
    let mut mpu = mpu.lock().await;
    mpu.init(&mut delay).expect("MPU6050 init failed");

    mpu.set_temp_enabled(true).expect("set_temp_enabled failed");
}

#[embassy_executor::task]
async fn check_orientation_and_vibrate(
    mpu: Arc<Mutex<CriticalSectionRawMutex, Mpu6050<I2c<'static, esp_hal::Blocking>>>>,
    vibration_motor: Arc<Mutex<CriticalSectionRawMutex, Output<'static>>>,
    rtc: Arc<Mutex<CriticalSectionRawMutex, Rtc<'static>>>,
) {
    // If the device is upside down and has been for 5 seconds, vibrate in increasing intervals and intensities
    // Otherwise the device is not upside down, turn off the vibration motor
    let mut upside_down_since: Option<Instant> = None;
    let sleep_duration = Duration::from_secs(SLEEP_DURATION_SECS.into());
    let vibration_duration = Duration::from_secs(2);
    let upside_down_threshold = Duration::from_secs(UPSIDE_DOWN_THRESHOLD_SECS.into());

    loop {
        let [_y_acc, _x_acc, z_acc] = {
            // Read the accelerometer data
            mpu.lock()
                .await
                .get_acc()
                .map(|x| x.into())
                .unwrap_or([0.0, 0.0, 0.0])
        };

        // Determine if the device is upside down
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
        if let Some(some_since) = upside_down_since {
            let upside_down_duration = Instant::now().duration_since(some_since);
            if upside_down_duration > upside_down_threshold {
                info!(
                    "Upside down for over {} seconds, vibrating",
                    upside_down_threshold.as_secs()
                );

                vibrate(
                    vibration_duration,
                    vibration_motor.clone(),
                    vibration_motor.clone(),
                )
                .await;

                upside_down_since = None;
            }
        }

        rtc.lock()
            .await
            .sleep_light(&[&TimerWakeupSource::new(sleep_duration.into())]);
    }
}
