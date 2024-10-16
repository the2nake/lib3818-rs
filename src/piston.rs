use vexide::prelude::AdiDigitalOut;

pub struct Piston {
    adi_out: AdiDigitalOut,
    activated: bool,
}

impl Piston {
    pub fn new(mut adi: AdiDigitalOut, initial: bool) -> Self {
        if initial {
            adi.set_high().ok();
        } else {
            adi.set_low().ok();
        }
        Piston {
            adi_out: adi,
            activated: initial,
        }
    }

    pub fn toggle(&mut self) {
        self.set(!self.activated);
    }

    pub fn set(&mut self, state: bool) {
        self.activated = state;
        self.update();
    }

    pub fn activated(&self) -> bool {
        self.activated
    }

    fn update(&mut self) {
        if self.activated {
            self.adi_out.set_high().ok();
        } else {
            self.adi_out.set_low().ok();
        }
    }
}
