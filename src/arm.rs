/*
use alloc::{collections::BTreeMap, string::String, vec::Vec};

struct State {
    name: String,
    action: fn(),
}

struct SwitchCondition {
    target_name: String,
    condition: fn() -> bool,
}

pub struct StateMachine {
    states: Vec<State>,
    conditions: BTreeMap<String, SwitchCondition>,
}

impl StateMachine {
    fn new() -> Self {
        let obj = StateMachine {

        };
        obj
    }
}
*/

use alloc::boxed::Box;

use vexide::prelude::*;

pub struct Arm {
    state: Option<Box<dyn ArmState>>,
    lift: Motor,
    wrist: Motor,
}

impl Arm {
    pub fn new(lift: Motor, wrist: Motor) -> Self {
        Arm {
            state: Some(Box::new(Starting {})),
            lift,
            wrist,
        }
    }

    pub fn act(&mut self) {
        // self.state.as_ref().unwrap().act();
    }

    pub fn update(&mut self) {
        if let Some(s) = self.state.take() {
            self.state = Some(s.update());
        }
    }
}

trait ArmState {
    fn act(&mut self);
    fn update(self: Box<Self>) -> Box<dyn ArmState>;
}

struct Starting {}

impl ArmState for Starting {
    fn act(&mut self) {}

    fn update(self: Box<Self>) -> Box<dyn ArmState> {
        Box::new(Ready {})
    }
}

struct Ready {}

impl ArmState for Ready {
    fn act(&mut self) {}

    fn update(self: Box<Self>) -> Box<dyn ArmState> {
        self // TODO: fix me
    }
}
