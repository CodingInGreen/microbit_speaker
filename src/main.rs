#![no_main]
#![no_std]

//Comment out test
//use defmt_rtt as _;
//use panic_halt as _;

//Add test
use rtt_target::{rprintln, rtt_init_print};
use panic_rtt_target as _;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use microbit::{
    hal::{
        clocks::Clocks,
        gpiote::Gpiote,
        prelude::OutputPin,
        gpio,
        pwm::{self, Pwm},
        rtc::{Rtc, RtcInterrupt},
        time::Hertz,
    },
    pac::{self, interrupt},
    Board,
};

static GPIO: Mutex<RefCell<Option<Gpiote>>> = Mutex::new(RefCell::new(None));
static SPEAKER: Mutex<RefCell<Option<Pwm<pac::PWM0>>>> = Mutex::new(RefCell::new(None));

const SOUND_A_FREQUENCY: u32 = 440; // Frequency for button A (A4 note)
const SOUND_B_FREQUENCY: u32 = 523; // Frequency for button B (C5 note)

#[entry]
fn main() -> ! {
    rtt_init_print!();
    
    let board = Board::take().unwrap();

    let gpiote = Gpiote::new(board.GPIOTE);

    let channel0 = gpiote.channel0();
    channel0
        .input_pin(&board.buttons.button_a.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel0.reset_events();

    let channel1 = gpiote.channel1();
    channel1
        .input_pin(&board.buttons.button_b.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel1.reset_events();

    // Speaker setup
    let mut speaker_pin = board.speaker_pin.into_push_pull_output(gpio::Level::High);
    let _ = speaker_pin.set_low();
    let speaker = Pwm::new(board.PWM0);
    speaker.set_output_pin(pwm::Channel::C0, speaker_pin.degrade())
           .set_prescaler(pwm::Prescaler::Div16)
           .set_period(Hertz(1u32))
           .set_counter_mode(pwm::CounterMode::UpAndDown)
           .set_max_duty(32767)
           .enable();
    speaker.set_seq_refresh(pwm::Seq::Seq0, 0)
           .set_seq_end_delay(pwm::Seq::Seq0, 0);
    let max_duty = speaker.max_duty();
    speaker.set_duty_on_common(max_duty / 2);


    cortex_m::interrupt::free(move |cs| {
        /* Enable external GPIO interrupts */
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::GPIOTE);
        }
        pac::NVIC::unpend(pac::Interrupt::GPIOTE);

        *GPIO.borrow(cs).borrow_mut() = Some(gpiote);
        *SPEAKER.borrow(cs).borrow_mut() = Some(speaker);

        rprintln!("Welcome to the buttons demo. Press buttons A or B for some sounds.");
    });

    loop {
        continue;
    }
}

// Define an interrupt, i.e. function to call when exception occurs. Here if we receive an
// interrupt from a button press, the function will be called
#[interrupt]
fn GPIOTE() {
    /* Enter critical section */
    cortex_m::interrupt::free(|cs| {
        if let (Some(gpiote), Some(speaker)) = 
        (
        GPIO.borrow(cs).borrow().as_ref(),
        SPEAKER.borrow(cs).borrow().as_ref(),
        ) 
        
        {
            let buttonapressed = gpiote.channel0().is_event_triggered();
            let buttonbpressed = gpiote.channel1().is_event_triggered();

            if buttonapressed {
                rprintln!("Button A pressed");
                speaker.set_period(Hertz(SOUND_A_FREQUENCY));
            }

            if buttonbpressed {
                rprintln!("Button B pressed");
                speaker.set_period(Hertz(SOUND_B_FREQUENCY));
            }

            /* Clear events */
            gpiote.channel0().reset_events();
            gpiote.channel1().reset_events();
            speaker.set_duty_on_common(speaker.max_duty() / 2);
        }
    });
}