use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Result for [`select_pin2`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Either<A, B> {
    /// First future finished first.
    First(A),
    /// Second future finished first.
    Second(B),
}

/// Future for the [`select_pin2`] function.
#[derive(Debug)]
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct SelectPin2<'a, 'b, 'c, 'd, Fut1, Fut2> {
    fut1: &'a mut Pin<&'b mut Option<Fut1>>,
    fut2: &'c mut Pin<&'d mut Option<Fut2>>,
}

/// Creates a new future which will select between two pinned optional futures of different types.
///
/// The returned future will wait for either future to be ready. Upon
/// completion the item resolved will be returned, wrapped in an Either enum
/// indicating which future was ready.
///
/// The completed future's Option will be set to None after completion.
pub fn select_pin2<'a, 'b, 'c, 'd, Fut1: Future, Fut2: Future>(
    fut1: &'a mut Pin<&'b mut Option<Fut1>>,
    fut2: &'c mut Pin<&'d mut Option<Fut2>>,
) -> SelectPin2<'a, 'b, 'c, 'd, Fut1, Fut2> {
    SelectPin2 { fut1, fut2 }
}

impl<'a, 'b, 'c, 'd, Fut1: Future, Fut2: Future> Future for SelectPin2<'a, 'b, 'c, 'd, Fut1, Fut2> {
    type Output = Either<Fut1::Output, Fut2::Output>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: Since `self` is pinned, the fields cannot move. Therefore it is safe
        // to access the fields and pin references to the contained futures.
        let this = unsafe { self.get_unchecked_mut() };

        // Poll the first future if it exists
        if let Some(fut1) = unsafe { this.fut1.as_mut().get_unchecked_mut().as_mut() } {
            match unsafe { Pin::new_unchecked(fut1) }.poll(cx) {
                Poll::Ready(output) => {
                    // Clear the future
                    *unsafe { this.fut1.as_mut().get_unchecked_mut() } = None;
                    return Poll::Ready(Either::First(output));
                }
                Poll::Pending => {}
            }
        }

        // Poll the second future if it exists
        if let Some(fut2) = unsafe { this.fut2.as_mut().get_unchecked_mut().as_mut() } {
            match unsafe { Pin::new_unchecked(fut2) }.poll(cx) {
                Poll::Ready(output) => {
                    // Clear the future
                    *unsafe { this.fut2.as_mut().get_unchecked_mut() } = None;
                    return Poll::Ready(Either::Second(output));
                }
                Poll::Pending => {}
            }
        }

        Poll::Pending
    }
}
