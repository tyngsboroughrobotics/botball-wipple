use botball::*;
use std::os::raw::*;
use wipple::*;
use wipple_bind::*;
use wipple_plugins::wipple_plugin;

#[wipple_plugin]
pub fn plugin(env: &EnvironmentRef, _stack: &Stack) -> Result {
    define_bindings!(env.borrow_mut(), {
        fn accel_x() -> c_short;
        fn accel_y() -> c_short;
        fn accel_z() -> c_short;
        fn accel_calibrate() -> c_int;
        fn analog(c_int) -> c_int;
        fn analog8(c_int) -> c_int;
        fn analog10(c_int) -> c_int;
        fn analog12(c_int) -> c_int;
        fn analog_et(c_int) -> c_int;
        fn set_analog_pullup(c_int, c_int);
        fn get_analog_pullup(c_int) -> c_int;
        fn beep();
        fn battery_charging() -> c_int;
        fn power_level() -> f32;
        fn power_level_nimh() -> f32;
        fn power_level_lipo() -> f32;
        fn power_level_life() -> f32;
        fn shut_down_in(f64);
        fn wait_for_light(c_int);
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
        fn set_extra_buttons_visible(c_int);
        fn camera_open() -> c_int;
        fn camera_open_black() -> c_int;
        fn set_camera_width(c_int);
        fn set_camera_height(c_int);
        fn get_camera_width() -> c_int;
        fn get_camera_height() -> c_int;
        fn camera_update() -> c_int;
        fn get_channel_count() -> c_int;
        fn get_object_count(c_int) -> c_int;
        fn get_code_num(c_int, c_int) -> c_int;
        fn get_object_data_length(c_int, c_int) -> c_int;
        fn get_object_confidence(c_int, c_int) -> f64;
        fn get_object_area(c_int, c_int) -> c_int;
        fn get_object_bbox_ulx(c_int, c_int) -> c_int;
        fn get_object_bbox_uly(c_int, c_int) -> c_int;
        fn get_object_bbox_brx(c_int, c_int) -> c_int;
        fn get_object_bbox_bry(c_int, c_int) -> c_int;
        fn get_object_bbox_width(c_int, c_int) -> c_int;
        fn get_object_bbox_height(c_int, c_int) -> c_int;
        fn get_object_centroid_column(c_int, c_int) -> c_int;
        fn get_object_centroid_x(c_int, c_int) -> c_int;
        fn get_object_centroid_row(c_int, c_int) -> c_int;
        fn get_object_centroid_y(c_int, c_int) -> c_int;
        fn get_object_center_column(c_int, c_int) -> c_int;
        fn get_object_center_x(c_int, c_int) -> c_int;
        fn get_object_center_row(c_int, c_int) -> c_int;
        fn get_object_center_y(c_int, c_int) -> c_int;
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
        fn create_demo(c_int);
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
        fn set_create_normalized_angle(c_int);
        fn get_create_total_angle() -> c_int;
        fn set_create_total_angle(c_int);
        fn get_create_distance() -> c_int;
        fn set_create_distance(c_int);
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
        fn create_drive(c_int, c_int);
        fn create_drive_straight(c_int);
        fn create_drive_direct(c_int, c_int);
        fn create_spin_block(c_int, c_int);
        fn create_advance_led(c_int);
        fn create_play_led(c_int);
        fn create_power_led(c_int, c_int);
        fn create_digital_output(c_int);
        fn create_pwm_low_side_drivers(c_int, c_int, c_int);
        fn create_low_side_drivers(c_int, c_int, c_int);
        fn create_load_song(c_int);
        fn create_play_song(c_int);
        fn create_write_byte(c_char);
        fn create_clear_serial_buffer();
        fn digital(c_int) -> c_int;
        fn set_digital_value(c_int, c_int);
        fn get_digital_value(c_int) -> c_int;
        fn set_digital_output(c_int, c_int);
        fn get_digital_output(c_int) -> c_int;
        fn get_digital_pullup(c_int) -> c_int;
        fn set_digital_pullup(c_int, c_int);
        fn display_clear();
        fn set_auto_publish(c_int);
        fn publish();
        fn halt();
        fn freeze_halt();
        fn graphics_open(c_int, c_int) -> c_int;
        fn graphics_close();
        fn graphics_update();
        fn graphics_clear();
        fn graphics_fill(c_int, c_int, c_int);
        fn graphics_pixel(c_int, c_int, c_int, c_int, c_int);
        fn graphics_line(c_int, c_int, c_int, c_int, c_int, c_int, c_int);
        fn graphics_circle(c_int, c_int, c_int, c_int, c_int, c_int);
        fn graphics_circle_fill(c_int, c_int, c_int, c_int, c_int, c_int);
        fn graphics_triangle(
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
        );
        fn graphics_triangle_fill(
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
            c_int,
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
        fn get_motor_position_counter(c_int) -> c_int;
        fn gmpc(c_int) -> c_int;
        fn clear_motor_position_counter(c_int);
        fn cmpc(c_int);
        fn move_at_velocity(c_int, c_int) -> c_int;
        fn mav(c_int, c_int) -> c_int;
        fn move_to_position(c_int, c_int, c_int) -> c_int;
        fn mtp(c_int, c_int, c_int) -> c_int;
        fn move_relative_position(c_int, c_int, c_int) -> c_int;
        fn mrp(c_int, c_int, c_int) -> c_int;
        fn set_pid_gains(
            c_int,
            c_short,
            c_short,
            c_short,
            c_short,
            c_short,
            c_short,
        );
        fn freeze(c_int) -> c_int;
        fn get_motor_done(c_int) -> c_int;
        fn block_motor_done(c_int);
        fn bmd(c_int);
        fn setpwm(c_int, c_int) -> c_int;
        fn getpwm(c_int) -> c_int;
        fn fd(c_int);
        fn bk(c_int);
        fn motor(c_int, c_int);
        fn baasbennaguui(c_int, c_int);
        fn motor_power(c_int, c_int);
        fn off(c_int);
        fn alloff();
        fn ao();
        fn get_robot_states_sequence_num() -> c_ulong;
        fn get_robot_update_count() -> c_ulong;
        fn get_robot_firmware_version() -> c_ushort;
        fn set_robot_update_delay(c_uint) -> c_int;
        fn get_robot_update_delay() -> c_uint;
        fn set_low_volt_threshold(f32) -> c_int;
        fn get_low_volt_threshold() -> f32;
        fn enable_servo(c_int);
        fn disable_servo(c_int);
        fn enable_servos();
        fn disable_servos();
        fn set_servo_enabled(c_int, c_int);
        fn get_servo_enabled(c_int) -> c_int;
        fn get_servo_position(c_int) -> c_int;
        fn set_servo_position(c_int, c_int);
        fn msleep(c_long);
        fn iitxash(c_long);
        fn systime() -> c_ulong;
        fn seconds() -> f64;
        fn wait_for_milliseconds(c_long);
        fn wait_for_touch(c_int);
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

    Ok(Value::empty())
}
