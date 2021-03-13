use bigdecimal::{BigDecimal, FromPrimitive, ToPrimitive};
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
        fn accel_x() -> i16;
        fn accel_y() -> i16;
        fn accel_z() -> i16;
        fn accel_calibrate() -> i32;
        fn analog(port: i32) -> i32;
        fn analog8(port: i32) -> i32;
        fn analog10(port: i32) -> i32;
        fn analog12(port: i32) -> i32;
        fn analog_et(port: i32) -> i32;
        fn set_analog_pullup(port: i32, pullup: i32);
        fn get_analog_pullup(port: i32) -> i32;
        fn beep();
        fn battery_charging() -> i32;
        fn power_level() -> f32;
        fn power_level_nimh() -> f32;
        fn power_level_lipo() -> f32;
        fn power_level_life() -> f32;
        fn shut_down_in(s: f64);
        fn wait_for_light(light_port_: i32);
        fn a_button() -> i32;
        fn b_button() -> i32;
        fn c_button() -> i32;
        fn x_button() -> i32;
        fn y_button() -> i32;
        fn z_button() -> i32;
        fn side_button() -> i32;
        fn black_button() -> i32;
        fn push_button() -> i32;
        fn left_button() -> i32;
        fn right_button() -> i32;
        fn a_button_clicked() -> i32;
        fn b_button_clicked() -> i32;
        fn c_button_clicked() -> i32;
        fn x_button_clicked() -> i32;
        fn y_button_clicked() -> i32;
        fn z_button_clicked() -> i32;
        fn side_button_clicked() -> i32;
        fn any_button() -> i32;
        fn extra_buttons_show();
        fn extra_buttons_hide();
        fn get_extra_buttons_visible() -> i32;
        fn set_extra_buttons_visible(visible: i32);
        fn camera_open() -> i32;
        fn camera_open_black() -> i32;
        fn set_camera_width(width: i32);
        fn set_camera_height(height: i32);
        fn get_camera_width() -> i32;
        fn get_camera_height() -> i32;
        fn camera_update() -> i32;
        fn get_channel_count() -> i32;
        fn get_object_count(channel: i32) -> i32;
        fn get_code_num(channel: i32, object: i32) -> i32;
        fn get_object_data_length(channel: i32, object: i32) -> i32;
        fn get_object_confidence(channel: i32, object: i32) -> f64;
        fn get_object_area(channel: i32, object: i32) -> i32;
        fn get_object_bbox_ulx(channel: i32, object: i32) -> i32;
        fn get_object_bbox_uly(channel: i32, object: i32) -> i32;
        fn get_object_bbox_brx(channel: i32, object: i32) -> i32;
        fn get_object_bbox_bry(channel: i32, object: i32) -> i32;
        fn get_object_bbox_width(channel: i32, object: i32) -> i32;
        fn get_object_bbox_height(channel: i32, object: i32) -> i32;
        fn get_object_centroid_column(channel: i32, object: i32) -> i32;
        fn get_object_centroid_x(channel: i32, object: i32) -> i32;
        fn get_object_centroid_row(channel: i32, object: i32) -> i32;
        fn get_object_centroid_y(channel: i32, object: i32) -> i32;
        fn get_object_center_column(channel: i32, object: i32) -> i32;
        fn get_object_center_x(channel: i32, object: i32) -> i32;
        fn get_object_center_row(channel: i32, object: i32) -> i32;
        fn get_object_center_y(channel: i32, object: i32) -> i32;
        fn camera_close();
        fn get_camera_element_size() -> u32;
        fn calibrate_compass();
        fn set_compass_params(
            mean_x: f32,
            mean_y: f32,
            mean_z: f32,
            w1: f32,
            w2: f32,
            div_e1: f32,
            div_e2: f32,
        );
        fn get_compass_angle() -> f32;
        fn console_clear();
        fn create_connect() -> i32;
        fn create_connect_once() -> i32;
        fn create_disconnect();
        fn create_start();
        fn create_passive();
        fn create_safe();
        fn create_full();
        fn create_spot();
        fn create_cover();
        fn create_demo(d: i32);
        fn create_cover_dock();
        fn get_create_mode() -> i32;
        fn get_create_lbump() -> i32;
        fn get_create_rbump() -> i32;
        fn get_create_lwdrop() -> i32;
        fn get_create_cwdrop() -> i32;
        fn get_create_rwdrop() -> i32;
        fn get_create_wall() -> i32;
        fn get_create_lcliff() -> i32;
        fn get_create_lfcliff() -> i32;
        fn get_create_rfcliff() -> i32;
        fn get_create_rcliff() -> i32;
        fn get_create_llightbump() -> i32;
        fn get_create_lflightbump() -> i32;
        fn get_create_lclightbump() -> i32;
        fn get_create_rclightbump() -> i32;
        fn get_create_rflightbump() -> i32;
        fn get_create_rlightbump() -> i32;
        fn get_create_llightbump_amt() -> i32;
        fn get_create_rlightbump_amt() -> i32;
        fn get_create_lflightbump_amt() -> i32;
        fn get_create_lclightbump_amt() -> i32;
        fn get_create_rclightbump_amt() -> i32;
        fn get_create_rflightbump_amt() -> i32;
        fn get_create_vwall() -> i32;
        fn get_create_overcurrents() -> i32;
        fn get_create_infrared() -> i32;
        fn get_create_advance_button() -> i32;
        fn get_create_play_button() -> i32;
        fn get_create_normalized_angle() -> i32;
        fn set_create_normalized_angle(angle: i32);
        fn get_create_total_angle() -> i32;
        fn set_create_total_angle(angle: i32);
        fn get_create_distance() -> i32;
        fn set_create_distance(dist: i32);
        fn get_create_battery_charging_state() -> i32;
        fn get_create_battery_voltage() -> i32;
        fn get_create_battery_current() -> i32;
        fn get_create_battery_temp() -> i32;
        fn get_create_battery_charge() -> i32;
        fn get_create_battery_capacity() -> i32;
        fn get_create_wall_amt() -> i32;
        fn get_create_lcliff_amt() -> i32;
        fn get_create_lfcliff_amt() -> i32;
        fn get_create_rfcliff_amt() -> i32;
        fn get_create_rcliff_amt() -> i32;
        fn get_create_bay_DI() -> i32;
        fn get_create_bay_AI() -> i32;
        fn get_create_song_number() -> i32;
        fn get_create_song_playing() -> i32;
        fn get_create_number_of_stream_packets() -> i32;
        fn get_create_requested_velocity() -> i32;
        fn get_create_requested_radius() -> i32;
        fn get_create_requested_right_velocity() -> i32;
        fn get_create_requested_left_velocity() -> i32;
        fn create_stop();
        fn create_drive(speed: i32, radius: i32);
        fn create_drive_straight(speed: i32);
        fn create_spin_CW(speed: i32);
        fn create_spin_CCW(speed: i32);
        fn create_drive_direct(l_speed: i32, r_speed: i32);
        fn create_spin_block(speed: i32, angle: i32);
        fn create_advance_led(on: i32);
        fn create_play_led(on: i32);
        fn create_power_led(color: i32, brightness: i32);
        fn create_digital_output(bits: i32);
        fn create_pwm_low_side_drivers(pwm2: i32, pwm1: i32, pwm0: i32);
        fn create_low_side_drivers(pwm2: i32, pwm1: i32, pwm0: i32);
        fn create_load_song(num: i32);
        fn create_play_song(num: i32);
        fn create_write_byte(byte: i8);
        fn create_clear_serial_buffer();
        fn digital(port: i32) -> i32;
        fn set_digital_value(port: i32, value: i32);
        fn get_digital_value(port: i32) -> i32;
        fn set_digital_output(port: i32, out: i32);
        fn get_digital_output(port: i32) -> i32;
        fn get_digital_pullup(port: i32) -> i32;
        fn set_digital_pullup(port: i32, pullup: i32);
        fn display_clear();
        fn set_auto_publish(on: i32);
        fn publish();
        fn halt();
        fn freeze_halt();
        fn graphics_open(width: i32, height: i32) -> i32;
        fn graphics_close();
        fn graphics_update();
        fn graphics_clear();
        fn graphics_fill(r: i32, g: i32, b: i32);
        fn graphics_pixel(x: i32, y: i32, r: i32, g: i32, b: i32);
        fn graphics_line(x1: i32, y1: i32, x2: i32, y2: i32, r: i32, g: i32, b: i32);
        fn graphics_circle(cx: i32, cy: i32, radius: i32, r: i32, g: i32, b: i32);
        fn graphics_circle_fill(cx: i32, cy: i32, radius: i32, r: i32, g: i32, b: i32);
        fn graphics_rectangle(x1: i32, y1: i32, x2: i32, y2: i32, r: i32, g: i32, b: i32);
        fn graphics_rectangle_fill(x1: i32, y1: i32, x2: i32, y2: i32, r: i32, g: i32, b: i32);
        fn graphics_triangle(
            x1: i32,
            y1: i32,
            x2: i32,
            y2: i32,
            x3: i32,
            y3: i32,
            r: i32,
            g: i32,
            b: i32,
        );
        fn graphics_triangle_fill(
            x1: i32,
            y1: i32,
            x2: i32,
            y2: i32,
            x3: i32,
            y3: i32,
            r: i32,
            g: i32,
            b: i32,
        );
        fn get_mouse_middle_button() -> i32;
        fn get_mouse_left_button() -> i32;
        fn get_mouse_right_button() -> i32;
        fn graphics_printCharacter(n: i32, x: i32, y: i32, r: i32, g: i32, b: i32, size: f32);
        fn g_printCharacter(n: i32, x: i32, y: i32, r: i32, g: i32, b: i32, size: f32);
        fn graphics_printInt(
            n: i32,
            min_num_digits: i32,
            x: i32,
            y: i32,
            r: i32,
            g: i32,
            b: i32,
            size: f32,
        ) -> i32;
        fn g_printInt(
            n: i32,
            min_num_digits: i32,
            x: i32,
            y: i32,
            r: i32,
            g: i32,
            b: i32,
            size: f32,
        ) -> i32;
        fn graphics_printFloat(
            n: f32,
            num_digits: i32,
            x: i32,
            y: i32,
            r: i32,
            g: i32,
            b: i32,
            size: f32,
        );
        fn g_printFloat(n: f32, num_digits: i32, x: i32, y: i32, r: i32, g: i32, b: i32, size: f32);
        fn gyro_x() -> i16;
        fn gyro_y() -> i16;
        fn gyro_z() -> i16;
        fn gyro_calibrate() -> i32;
        fn ir_read();
        fn ir_write();
        fn magneto_x() -> i16;
        fn magneto_y() -> i16;
        fn magneto_z() -> i16;
        fn magneto_calibrate() -> i32;
        fn get_motor_position_counter(motor: i32) -> i32;
        fn gmpc(motor: i32) -> i32;
        fn clear_motor_position_counter(motor: i32);
        fn cmpc(motor: i32);
        fn move_at_velocity(motor: i32, velocity: i32) -> i32;
        fn mav(motor: i32, velocity: i32) -> i32;
        fn move_to_position(motor: i32, speed: i32, goal_pos: i32) -> i32;
        fn mtp(motor: i32, speed: i32, goal_pos: i32) -> i32;
        fn move_relative_position(motor: i32, speed: i32, delta_pos: i32) -> i32;
        fn mrp(motor: i32, speed: i32, delta_pos: i32) -> i32;
        fn set_pid_gains(motor: i32, p: i16, i: i16, d: i16, pd: i16, id: i16, dd: i16);
        fn freeze(motor: i32) -> i32;
        fn get_motor_done(motor: i32) -> i32;
        fn block_motor_done(motor: i32);
        fn bmd(motor: i32);
        fn setpwm(motor: i32, pwm: i32) -> i32;
        fn getpwm(motor: i32) -> i32;
        fn fd(motor: i32);
        fn bk(motor: i32);
        fn motor(motor: i32, percent: i32);
        fn baasbennaguui(motor: i32, percent: i32);
        fn motor_power(motor: i32, percent: i32);
        fn off(motor: i32);
        fn alloff();
        fn ao();
        fn get_robot_states_sequence_num() -> u64;
        fn get_robot_update_count() -> u64;
        fn get_robot_firmware_version() -> u16;
        fn set_robot_update_delay(us_delay: u32) -> i32;
        fn get_robot_update_delay() -> u32;
        fn set_low_volt_threshold(volts: f32) -> i32;
        fn get_low_volt_threshold() -> f32;
        fn enable_servo(port: i32);
        fn disable_servo(port: i32);
        fn enable_servos();
        fn disable_servos();
        fn set_servo_enabled(port: i32, enabled: i32);
        fn get_servo_enabled(port: i32) -> i32;
        fn get_servo_position(port: i32) -> i32;
        fn set_servo_position(port: i32, position: i32);
        fn msleep(msecs: i64);
        fn iitxash(msecs: i64);
        fn systime() -> u64;
        fn seconds() -> f64;
        fn wait_for_milliseconds(msecs: i64);
        fn wait_for_touch(port: i32);
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
