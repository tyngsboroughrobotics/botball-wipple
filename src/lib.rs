use botball::*;
use wipple::*;
use wipple_plugin_loader::wipple_plugin;

wipple_plugin!(|env: &mut Environment, _: &Stack| {
    env.set_variable(
        "factory-test!",
        computed_builtin(|_, _| {
            unsafe { test() };

            Ok(Value::empty())
        }),
    );

    Ok(Value::empty())
});

fn computed_builtin(evaluate: impl Fn(&mut Environment, &Stack) -> Result + 'static) -> Value {
    Value::of(EvaluateFn::new(evaluate)).add(&Trait {
        id: TraitID::computed(),
        value: Value::of(Computed),
    })
}

unsafe fn test() {
    // Drive both wheels forward at full speed for 1.5 seconds
    motor(0, 100);
    motor(1, 100);
    msleep(1500);
    stop(0);
    stop(1);

    // Wait 1.5 seconds before going backward
    msleep(1500);

    // Drive both wheels backward at full speed for 1.5 seconds
    motor(0, -100);
    motor(1, -100);
    msleep(1500);
    stop(0);
    stop(1);
}

unsafe fn stop(motor_port: i32) {
    motor(motor_port, 0);
    off(motor_port);
}
