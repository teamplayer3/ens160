#[derive(Debug)]
pub struct AirQualityConvError(pub u16);

#[cfg(feature = "std")]
impl std::fmt::Display for AirQualityConvError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} not a valid air quality reading (400..).", self.0)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AirQualityConvError {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{} not a valid air quality reading (400..).", self.0)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for AirQualityConvError {}
