#[derive(Clone, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub enum Option<T> {
    /// No value
    None,
    /// Some value `T`
    Some(T)
}
