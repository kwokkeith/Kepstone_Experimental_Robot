mod vacuum;
mod pwm_init;
mod sidebrush;
mod servo;
mod bts7960_motor_control;

fn main() {
    sidebrush::main();
    //vacuum::main();
    //pwm_init::main();
}
