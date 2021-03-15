use bigdecimal::{BigDecimal, FromPrimitive, ToPrimitive};
use std::os::raw::*;
use wipple::*;
use wipple_plugins::wipple_plugin;

wipple_plugin!(|env: &EnvironmentRef, _: &Stack| {
    add_bindings(env);

    Ok(Value::empty())
});

trait ConvertValue
where
    Self: Sized,
{
    fn from_value(value: &Value, env: &EnvironmentRef, stack: &Stack) -> Result<Self>;
    fn into_value(self, env: &EnvironmentRef, stack: &Stack) -> Result;
}

macro_rules! bind {
    ($env:ident, { $(fn $func:ident($($arg:ident: $argtype:ty),* $(,)?) $(-> $returntype:ty)?);* $(;)? }) => {
        $(bind!(fn $func($($arg: $argtype),*) $(-> $returntype)?, $env);)*
    };
    (fn $func:ident(), $env:ident) => {
        bind!(fn $func() -> (), $env);
    };
    (fn $func:ident() -> $returntype:ty, $env:ident) => {
        $env.borrow_mut().set_variable(
            stringify!($func),
            computed_builtin(|env, stack| {
                let result: $returntype = unsafe { botball::$func() };
                ConvertValue::into_value(result, env, stack)
            }),
        )
    };
    (fn $func:ident($($args:ident: $argtypes:ty),* $(,)?), $env:ident) => {
        bind!(fn $func($($args: $argtypes),*) -> (), $env)
    };
    (fn $func:ident($arg:ident: $argtype:ty $(, $restargs:ident: $restargtypes:ty)* $(,)?) -> $returntype:ty, $env:ident) => {
        $env.borrow_mut()
            .set_variable(stringify!($func), Value::of(Function::new(move |value, env, stack| {
                let $arg: $argtype = ConvertValue::from_value(value, env, stack)?;
                bind!(@expand fn $func($($restargs: $restargtypes),*) -> $returntype, [$arg], env, stack)
            })));
    };
    (@expand fn $func:ident($arg:ident: $argtype:ty $(, $restargs:ident: $restargtypes:ty)*) -> $returntype:ty, [$($var:ident),+], $env:ident, $stack:expr) => {
        Ok(Value::of(Function::new(move |value, env, stack| {
            let $arg: $argtype = ConvertValue::from_value(value, env, stack)?;
            bind!(@expand fn $func($($restargs: $restargtypes),*) -> $returntype, [$($var),+, $arg], env, stack)
        })))
    };
    (@expand fn $func:ident() -> $returntype:ty, [$($var:ident),+], $env:ident, $stack:expr) => {{
        let result: $returntype = unsafe { botball::$func($($var),+) };
        ConvertValue::into_value(result, $env, $stack)
    }};
}

fn computed_builtin(evaluate: impl Fn(&EnvironmentRef, &Stack) -> Result + 'static) -> Value {
    Value::of(EvaluateFn::new(evaluate)).add(&Trait::of_primitive(Computed))
}

impl ConvertValue for () {
    fn from_value(value: &Value, _: &EnvironmentRef, stack: &Stack) -> Result<Self> {
        if value.is_empty() {
            Ok(())
        } else {
            Err(ReturnState::Error(Error::new(
                "Expected empty value",
                stack,
            )))
        }
    }

    fn into_value(self, _: &EnvironmentRef, _: &Stack) -> Result {
        Ok(Value::empty())
    }
}

macro_rules! number_conversion {
    ($type:ty, $desc:expr) => {
        paste::paste! {
            impl ConvertValue for $type {
                fn from_value(value: &Value, env: &EnvironmentRef, stack: &Stack) -> Result<Self> {
                    let number = value.get_primitive_or::<Number>("Expected number", env, stack)?;

                    number.number.[<to_ $type>]().ok_or_else(|| {
                        ReturnState::Error(Error::new(
                            &format!("Number cannot be represented as a {}", $desc),
                            stack,
                        ))
                    })
                }

                fn into_value(self, _: &EnvironmentRef, stack: &Stack) -> Result {
                    let number = BigDecimal::[<from_ $type>](self).ok_or_else(|| {
                        ReturnState::Error(Error::new(
                            &format!("Number cannot be represented by a {}", $desc),
                            stack,
                        ))
                    })?;

                    Ok(Value::of(Number::new(number)))
                }
            }
        }
    };
}

number_conversion!(i8, "8-bit integer");
number_conversion!(i16, "16-bit integer");
number_conversion!(i32, "32-bit integer");
number_conversion!(i64, "64-bit integer");
number_conversion!(i128, "128-bit integer");
number_conversion!(u8, "8-bit unsigned integer");
number_conversion!(u16, "16-bit unsigned integer");
number_conversion!(u32, "32-bit unsigned integer");
number_conversion!(u64, "64-bit unsigned integer");
number_conversion!(u128, "128-bit unsigned integer");
number_conversion!(f32, "32-bit floating point number");
number_conversion!(f64, "64-bit floating point number");

fn add_bindings(env: &EnvironmentRef) {
    bind!(env, {
        fn accel_x() -> c_short;
        fn accel_y() -> c_short;
        fn accel_z() -> c_short;
        fn accel_calibrate() -> c_int;
        fn analog(port: c_int) -> c_int;
        fn analog8(port: c_int) -> c_int;
        fn analog10(port: c_int) -> c_int;
        fn analog12(port: c_int) -> c_int;
        fn analog_et(port: c_int) -> c_int;
        fn set_analog_pullup(port: c_int, pullup: c_int);
        fn get_analog_pullup(port: c_int) -> c_int;
        fn beep();
        fn battery_charging() -> c_int;
        fn power_level() -> f32;
        fn power_level_nimh() -> f32;
        fn power_level_lipo() -> f32;
        fn power_level_life() -> f32;
        fn shut_down_in(s: f64);
        fn wait_for_light(light_port_: c_int);
        fn a_button() -> c_int;
        fn b_button() -> c_int;
        fn c_button() -> c_int;
        fn x_button() -> c_int;
        fn y_button() -> c_int;
        fn z_button() -> c_int;
        fn side_button() -> c_int;
        fn black_button() -> c_int;
        fn push_button() -> c_int;
        fn left_button() -> c_int;
        fn right_button() -> c_int;
        fn a_button_clicked() -> c_int;
        fn b_button_clicked() -> c_int;
        fn c_button_clicked() -> c_int;
        fn x_button_clicked() -> c_int;
        fn y_button_clicked() -> c_int;
        fn z_button_clicked() -> c_int;
        fn side_button_clicked() -> c_int;
        fn any_button() -> c_int;
        fn extra_buttons_show();
        fn extra_buttons_hide();
        fn get_extra_buttons_visible() -> c_int;
        fn set_extra_buttons_visible(visible: c_int);
        fn camera_open() -> c_int;
        fn camera_open_black() -> c_int;
        fn set_camera_width(width: c_int);
        fn set_camera_height(height: c_int);
        fn get_camera_width() -> c_int;
        fn get_camera_height() -> c_int;
        fn camera_update() -> c_int;
        fn get_channel_count() -> c_int;
        fn get_object_count(channel: c_int) -> c_int;
        fn get_code_num(channel: c_int, object: c_int) -> c_int;
        fn get_object_data_length(channel: c_int, object: c_int) -> c_int;
        fn get_object_confidence(channel: c_int, object: c_int) -> f64;
        fn get_object_area(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_ulx(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_uly(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_brx(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_bry(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_width(channel: c_int, object: c_int) -> c_int;
        fn get_object_bbox_height(channel: c_int, object: c_int) -> c_int;
        fn get_object_centroid_column(channel: c_int, object: c_int) -> c_int;
        fn get_object_centroid_x(channel: c_int, object: c_int) -> c_int;
        fn get_object_centroid_row(channel: c_int, object: c_int) -> c_int;
        fn get_object_centroid_y(channel: c_int, object: c_int) -> c_int;
        fn get_object_center_column(channel: c_int, object: c_int) -> c_int;
        fn get_object_center_x(channel: c_int, object: c_int) -> c_int;
        fn get_object_center_row(channel: c_int, object: c_int) -> c_int;
        fn get_object_center_y(channel: c_int, object: c_int) -> c_int;
        fn camera_close();
        fn get_camera_element_size() -> c_uint;
        fn calibrate_compass();
        fn get_compass_angle() -> f32;
        fn console_clear();
        fn create_connect() -> c_int;
        fn create_connect_once() -> c_int;
        fn create_disconnect();
        fn create_start();
        fn create_passive();
        fn create_safe();
        fn create_full();
        fn create_spot();
        fn create_cover();
        fn create_demo(d: c_int);
        fn create_cover_dock();
        fn get_create_mode() -> c_int;
        fn get_create_lbump() -> c_int;
        fn get_create_rbump() -> c_int;
        fn get_create_lwdrop() -> c_int;
        fn get_create_cwdrop() -> c_int;
        fn get_create_rwdrop() -> c_int;
        fn get_create_wall() -> c_int;
        fn get_create_lcliff() -> c_int;
        fn get_create_lfcliff() -> c_int;
        fn get_create_rfcliff() -> c_int;
        fn get_create_rcliff() -> c_int;
        fn get_create_llightbump() -> c_int;
        fn get_create_lflightbump() -> c_int;
        fn get_create_lclightbump() -> c_int;
        fn get_create_rclightbump() -> c_int;
        fn get_create_rflightbump() -> c_int;
        fn get_create_rlightbump() -> c_int;
        fn get_create_llightbump_amt() -> c_int;
        fn get_create_rlightbump_amt() -> c_int;
        fn get_create_lflightbump_amt() -> c_int;
        fn get_create_lclightbump_amt() -> c_int;
        fn get_create_rclightbump_amt() -> c_int;
        fn get_create_rflightbump_amt() -> c_int;
        fn get_create_vwall() -> c_int;
        fn get_create_overcurrents() -> c_int;
        fn get_create_infrared() -> c_int;
        fn get_create_advance_button() -> c_int;
        fn get_create_play_button() -> c_int;
        fn get_create_normalized_angle() -> c_int;
        fn set_create_normalized_angle(angle: c_int);
        fn get_create_total_angle() -> c_int;
        fn set_create_total_angle(angle: c_int);
        fn get_create_distance() -> c_int;
        fn set_create_distance(dist: c_int);
        fn get_create_battery_charging_state() -> c_int;
        fn get_create_battery_voltage() -> c_int;
        fn get_create_battery_current() -> c_int;
        fn get_create_battery_temp() -> c_int;
        fn get_create_battery_charge() -> c_int;
        fn get_create_battery_capacity() -> c_int;
        fn get_create_wall_amt() -> c_int;
        fn get_create_lcliff_amt() -> c_int;
        fn get_create_lfcliff_amt() -> c_int;
        fn get_create_rfcliff_amt() -> c_int;
        fn get_create_rcliff_amt() -> c_int;
        fn get_create_song_number() -> c_int;
        fn get_create_song_playing() -> c_int;
        fn get_create_number_of_stream_packets() -> c_int;
        fn get_create_requested_velocity() -> c_int;
        fn get_create_requested_radius() -> c_int;
        fn get_create_requested_right_velocity() -> c_int;
        fn get_create_requested_left_velocity() -> c_int;
        fn create_stop();
        fn create_drive(speed: c_int, radius: c_int);
        fn create_drive_straight(speed: c_int);
        fn create_drive_direct(l_speed: c_int, r_speed: c_int);
        fn create_spin_block(speed: c_int, angle: c_int);
        fn create_advance_led(on: c_int);
        fn create_play_led(on: c_int);
        fn create_power_led(color: c_int, brightness: c_int);
        fn create_digital_output(bits: c_int);
        fn create_pwm_low_side_drivers(pwm2: c_int, pwm1: c_int, pwm0: c_int);
        fn create_low_side_drivers(pwm2: c_int, pwm1: c_int, pwm0: c_int);
        fn create_load_song(num: c_int);
        fn create_play_song(num: c_int);
        fn create_write_byte(byte: c_char);
        fn create_clear_serial_buffer();
        fn digital(port: c_int) -> c_int;
        fn set_digital_value(port: c_int, value: c_int);
        fn get_digital_value(port: c_int) -> c_int;
        fn set_digital_output(port: c_int, out: c_int);
        fn get_digital_output(port: c_int) -> c_int;
        fn get_digital_pullup(port: c_int) -> c_int;
        fn set_digital_pullup(port: c_int, pullup: c_int);
        fn display_clear();
        fn set_auto_publish(on: c_int);
        fn publish();
        fn halt();
        fn freeze_halt();
        fn graphics_open(width: c_int, height: c_int) -> c_int;
        fn graphics_close();
        fn graphics_update();
        fn graphics_clear();
        fn graphics_fill(r: c_int, g: c_int, b: c_int);
        fn graphics_pixel(x: c_int, y: c_int, r: c_int, g: c_int, b: c_int);
        fn graphics_line(x1: c_int, y1: c_int, x2: c_int, y2: c_int, r: c_int, g: c_int, b: c_int);
        fn graphics_circle(cx: c_int, cy: c_int, radius: c_int, r: c_int, g: c_int, b: c_int);
        fn graphics_circle_fill(cx: c_int, cy: c_int, radius: c_int, r: c_int, g: c_int, b: c_int);
        fn graphics_triangle(
            x1: c_int,
            y1: c_int,
            x2: c_int,
            y2: c_int,
            x3: c_int,
            y3: c_int,
            r: c_int,
            g: c_int,
            b: c_int,
        );
        fn graphics_triangle_fill(
            x1: c_int,
            y1: c_int,
            x2: c_int,
            y2: c_int,
            x3: c_int,
            y3: c_int,
            r: c_int,
            g: c_int,
            b: c_int,
        );
        fn get_mouse_middle_button() -> c_int;
        fn get_mouse_left_button() -> c_int;
        fn get_mouse_right_button() -> c_int;
        fn gyro_x() -> c_short;
        fn gyro_y() -> c_short;
        fn gyro_z() -> c_short;
        fn gyro_calibrate() -> c_int;
        fn ir_read();
        fn ir_write();
        fn magneto_x() -> c_short;
        fn magneto_y() -> c_short;
        fn magneto_z() -> c_short;
        fn magneto_calibrate() -> c_int;
        fn get_motor_position_counter(motor: c_int) -> c_int;
        fn gmpc(motor: c_int) -> c_int;
        fn clear_motor_position_counter(motor: c_int);
        fn cmpc(motor: c_int);
        fn move_at_velocity(motor: c_int, velocity: c_int) -> c_int;
        fn mav(motor: c_int, velocity: c_int) -> c_int;
        fn move_to_position(motor: c_int, speed: c_int, goal_pos: c_int) -> c_int;
        fn mtp(motor: c_int, speed: c_int, goal_pos: c_int) -> c_int;
        fn move_relative_position(motor: c_int, speed: c_int, delta_pos: c_int) -> c_int;
        fn mrp(motor: c_int, speed: c_int, delta_pos: c_int) -> c_int;
        fn set_pid_gains(
            motor: c_int,
            p: c_short,
            i: c_short,
            d: c_short,
            pd: c_short,
            id: c_short,
            dd: c_short,
        );
        fn freeze(motor: c_int) -> c_int;
        fn get_motor_done(motor: c_int) -> c_int;
        fn block_motor_done(motor: c_int);
        fn bmd(motor: c_int);
        fn setpwm(motor: c_int, pwm: c_int) -> c_int;
        fn getpwm(motor: c_int) -> c_int;
        fn fd(motor: c_int);
        fn bk(motor: c_int);
        fn motor(motor: c_int, percent: c_int);
        fn baasbennaguui(motor: c_int, percent: c_int);
        fn motor_power(motor: c_int, percent: c_int);
        fn off(motor: c_int);
        fn alloff();
        fn ao();
        fn get_robot_states_sequence_num() -> c_ulong;
        fn get_robot_update_count() -> c_ulong;
        fn get_robot_firmware_version() -> c_ushort;
        fn set_robot_update_delay(us_delay: c_uint) -> c_int;
        fn get_robot_update_delay() -> c_uint;
        fn set_low_volt_threshold(volts: f32) -> c_int;
        fn get_low_volt_threshold() -> f32;
        fn enable_servo(port: c_int);
        fn disable_servo(port: c_int);
        fn enable_servos();
        fn disable_servos();
        fn set_servo_enabled(port: c_int, enabled: c_int);
        fn get_servo_enabled(port: c_int) -> c_int;
        fn get_servo_position(port: c_int) -> c_int;
        fn set_servo_position(port: c_int, position: c_int);
        fn msleep(msecs: c_long);
        fn iitxash(msecs: c_long);
        fn systime() -> c_ulong;
        fn seconds() -> f64;
        fn wait_for_milliseconds(msecs: c_long);
        fn wait_for_touch(port: c_int);
        fn wait_for_a_button();
        fn wait_for_b_button();
        fn wait_for_c_button();
        fn wait_for_x_button();
        fn wait_for_y_button();
        fn wait_for_z_button();
        fn wait_for_side_button();
        fn wait_for_any_button();
        fn wait_for_a_button_clicked();
        fn wait_for_b_button_clicked();
        fn wait_for_c_button_clicked();
        fn wait_for_x_button_clicked();
        fn wait_for_y_button_clicked();
        fn wait_for_z_button_clicked();
        fn wait_for_side_button_clicked();
    });
}
