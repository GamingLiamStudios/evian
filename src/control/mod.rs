//! Control Theory Primitives
//!
//! This module provides basic building-blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

use core::time::Duration;

pub mod pid;

/// A trait for implementing a Controller for a System
///
/// Controllers are algorithms that attempt to reach a desired state (setpoint) by using logic
/// determined by the designer to bring a System towards the desired state.
///
/// There are typically 2 designs used to control a system;
/// # Closed-loop Design
/// Otherwise known as Feedback Control, this design uses a measured state to determine what
/// the future input should be to bring the System to the desired state.
///
/// An example of a Simple P (proportional only) Feedback controller:
///
/// ```
/// use core::time::Duration;
///
/// struct FeedbackController {
///     setpoint: f64,
///     gain: f64,
/// }
///
/// impl Controller for FeedbackController {
///     type Output = f64;
///     type Input = f64;
///
///     fn update(&mut self, measured: Self::Output, _dt: Duration) -> Self::Output {
///         self.gain * (self.setpoint - self.measured)
///     }
///
///     fn set_desired(&mut self, setpoint: Self::Output) {
///         self.setpoint = setpoint;
///     }
/// }
/// ```
///
/// # Open-loop Design
/// Otherwise known as Feed-forward Control, this design instead determines the System Input
/// purely from the desired state.
///
/// An example of a Simple Feedforward controller:
///
/// ```
/// use core::time::Duration;
///
/// struct FeedforwardController {
///     setpoint: f64,
///     gain: f64,
/// }
///
/// impl Feedback for FeedforwardController {
///     type Error = f64;
///     type Output = f64;
///
///     fn update(&mut self, _measured: Self::Error, _dt: Duration) -> Self::Output {
///         self.gain * self.setpoint
///     }
///
///     fn set_desired(&mut self, setpoint: Self::Output) {
///         self.setpoint = setpoint;
///     }
/// }
/// ```
pub trait Controller {
    /// The controller's input type
    ///
    /// The output is whatever can be "measured" about your system. For example, this could be the velocity
    /// of a Motor, or the global position on a field.
    type Output;

    /// The controller's input type.
    ///
    /// The input is whatever values control the behaviour of your system.
    type Input;

    /// Computes a new control signal given a new system measurement.
    ///
    /// This function also requires a `dt` ("delta time") value to be passed, which informs the
    /// controller of the amount of time that has elapsed since the last control signal was computed.
    fn update(&mut self, measured: Self::Output, dt: Duration) -> Self::Input;

    /// Updates the Controller's Target state.
    fn set_desired(&mut self, setpoint: Self::Output);
}
