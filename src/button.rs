pub const WAIT_TIME: usize = 400;
pub const DEBOUNCE_TIME: usize = 50;
pub const HOLD_TIME: usize = 800;

pub struct Button<T: Fn()> {
    clicks: usize,
    start_time: usize,
    single_click: Option<T>,
    double_click: Option<T>,
    triple_click: Option<T>,
    hold: Option<T>,
}

impl<T> Button<T> where T: Fn() {
    pub fn new() -> Self {
        Button {
            clicks: 0,
            start_time: 0,
            single_click: None,
            double_click: None,
            triple_click: None,
            hold: None,
        }
    }

    pub fn attach_single_click(&self, func: T) {
        self.single_click = Some(func);
    }

    pub fn attach_double_click(&self, func: T) {
        self.double_click = Some(func);
    }

    pub fn attach_triple_click(&self, func: T) {
        self.triple_click = Some(func);
    }

    pub fn attach_hold(&self, func: T) {
        self.hold = Some(func);
    }
}