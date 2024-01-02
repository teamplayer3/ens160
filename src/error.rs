#[derive(Debug)]
pub struct AirQualityConvError(pub(crate) u16);

#[cfg(feature = "std")]
impl std::fmt::Display for AirQualityConvError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} not a valid air quality reading (400..).", self.0)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for AirQualityConvError {}
