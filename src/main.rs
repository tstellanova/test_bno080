#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]



extern crate panic_itm;
//extern crate panic_semihosting;

use rtfm::app;
use rtfm::cyccnt::U32Ext;

use stm32f4xx_hal as processor_hal;

//use processor_hal::gpio::{gpiob::PB6, gpiob::PB7};
use processor_hal::gpio::{gpioc::PC13, Output, PushPull};
use processor_hal::prelude::*;
//use processor_hal::flash::FlashExt;
//use processor_hal::rcc::RccExt;
//use processor_hal::pwr::PwrExt;

//use processor_hal::i2c::{I2cExt};

use processor_hal::stm32 as pac;
use pac::I2C1;

use pac::DWT;


use embedded_hal::{
    digital::v2::{OutputPin, ToggleableOutputPin},
//    blocking::i2c::{Read, Write, WriteRead},
};

//use cortex_m_semihosting::{ hprintln};
use cortex_m;
use cortex_m::{iprintln};


use bno080::*;
//use cortex_m::peripheral::itm::Stim;

const BLINK_PERIOD: u32 = 1_000_000;
const IMU_READ_PERIOD: u32 = 10_000;

type ImuDriverType = bno080::BNO080<processor_hal::i2c::I2c<I2C1,
    (processor_hal::gpio::gpiob::PB6<processor_hal::gpio::Alternate<processor_hal::gpio::AF4>>,
     processor_hal::gpio::gpiob::PB7<processor_hal::gpio::Alternate<processor_hal::gpio::AF4>>)
>>;

//let gpioc = dp.GPIOC.split();
//let mut user_led1 = gpioc.pc13.into_push_pull_output();

// We need to pass monotonic = rtfm::cyccnt::CYCCNT to use schedule feature of RTFM
#[app(device = stm32f4xx_hal::stm32,  peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    // Global resources (global variables) are defined here and initialized with the
    // `LateResources` struct in init
    struct Resources {
        delay_source: processor_hal::delay::Delay,
        user_led1: PC13<Output<PushPull>>,
        i2c1_driver: ImuDriverType,
        itm: cortex_m::peripheral::ITM,
    }

    /// First stage startup: interrupts are disabled
    #[init(spawn=[kicker], schedule=[blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        // Note that interrupts are disabled in `init`

        // Enable cycle counter
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();
//        let before = core.DWT.cyccnt.read();

        let dp: processor_hal::stm32::Peripherals = cx.device;
        let cp = cortex_m::Peripherals::take().unwrap();

        // Setup clocks
        //let _flash = device.FLASH.constrain();


        // Constrain and Freeze clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        // source for delays
        let delay =  processor_hal::delay::Delay::new(cp.SYST, clocks);

        // Setup LED
        let gpioc = dp.GPIOC.split();
        let mut led1 = gpioc.pc13.into_push_pull_output();
        led1.set_high().unwrap();

        // Schedule the blinking task
        cx.schedule.blinker(cx.start + BLINK_PERIOD.cycles()).unwrap();

        // setup the BNO080 imu device
        // On stm32f401CxUx board, use pins PB6 (I2C_1_SCL) and PB7 (I2C_1_SDA)
        // (PB8 = I2C_1_SCL, PB9 = I2C_1_SDA)
        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb6.into_alternate_af4().internal_pull_up(true).set_open_drain();
        let sda = gpiob.pb7.into_alternate_af4().internal_pull_up(true).set_open_drain();
        let i2c_dev = processor_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);
        let i2c1_driver = BNO080::new(i2c_dev);
        
        cx.spawn.kicker().unwrap();

        init::LateResources {
            delay_source: delay,
            user_led1: led1,
            i2c1_driver: i2c1_driver,
            itm: cp.ITM,
        }

    }

//    #[idle]
//    fn idle(cx: idle::Context) -> ! {
//        // interrupts are enabled in `idle`
//        hprintln!("| {} | idle start: {}", DWT::get_cycle_count(), *cur_iterations ).unwrap();
//        loop {
//            rtfm::export::wfi();
//        }
//    }

    /// Second phase startup: interrupts are enabled
    #[task(resources = [i2c1_driver, delay_source, itm], spawn = [oneshot], schedule = [imu_reader]) ]
    fn kicker(cx: kicker::Context) {
        //let stim = &mut cortex_m::Peripherals::take().unwrap().ITM.stim[0];
        let stim =  &mut cx.resources.itm.stim[0];
        iprintln!(stim, "| {} | kicker start", DWT::get_cycle_count() );
        let res =  cx.resources.i2c1_driver.init(cx.resources.delay_source);
        if res.is_err() {
            iprintln!( stim,"bno080 init err {:?}", res);
        }
        else {
            //iprintln!(stim,"bno080 OK");
            cx.schedule.imu_reader(cx.scheduled + IMU_READ_PERIOD.cycles() ).unwrap();
        }

        cx.spawn.oneshot().unwrap();
        iprintln!(stim,"| {} | kicker done", DWT::get_cycle_count() );
    }

    #[task(resources = [i2c1_driver], schedule = [imu_reader])]
    fn imu_reader(cx: imu_reader::Context) {
        cx.resources.i2c1_driver.handle_all_messages();
        let sched_res = cx.schedule.imu_reader(cx.scheduled + IMU_READ_PERIOD.cycles());
        if sched_res.is_err() {
            //iprintln!(stim,"sched err: {:?}", sched_res);
        }
    }


    #[task]
    fn oneshot(_cx: oneshot::Context) {
        //hprintln!("| {} | oneshot done",DWT::get_cycle_count() ).unwrap();
    }

    #[task(resources = [user_led1], schedule = [blinker])]
    fn blinker(cx: blinker::Context) {
        // Use the safe local `static mut` of RTFM
        static mut LED_STATE: bool = false;

        //iprintln!(stim, "| {} | blinker start", DWT::get_cycle_count() );

        if *LED_STATE {
            cx.resources.user_led1.toggle().unwrap();
            *LED_STATE = false;
        }
        else {
            cx.resources.user_led1.toggle().unwrap();
            *LED_STATE = true;
        }
        let sched_res = cx.schedule.blinker(cx.scheduled + BLINK_PERIOD.cycles());
        if sched_res.is_err() {
            //iprintln!(stim,"sched err: {:?}", sched_res);
        }

    }

//    #[task(binds = I2C1_EV, priority = 2, )]
//    fn i2c1_ev(_cx: i2c1_ev::Context) {
//        // send data to the handler
//        hprintln!("| {} | I2C1_EV",DWT::get_cycle_count() ).unwrap();
//    }

    // define a list of free/unused interrupts that rtfm may utilize
    // for dispatching software tasks
    extern "C" {
        //fn EXTI0();
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};





