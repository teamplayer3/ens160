#[derive(Debug)]
pub struct AirqualityConvError(pub(crate) u16);

#[cfg(feature = "std")]
impl std::fmt::Display for AirqualityConvError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} is no valid qirquality value. Values start from 400.",
            self.0
        )
    }
}

#[cfg(feature = "std")]
impl std::error::Error for AirqualityConvError {}
