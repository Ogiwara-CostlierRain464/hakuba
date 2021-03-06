use std::sync::Arc;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering;

use std::thread;

fn main(){
    let spinlock = Arc::new(AtomicUsize::new(1));

    let spinlock_clone = Arc::clone(&spinlock);
    let thread = thread::spawn(move|| {
        spinlock_clone.store(0, Ordering::SeqCst);
    });

    while spinlock.load(Ordering::SeqCst) != 0 {}

    if let Err(panic) = thread.join(){
        println!("Thread had an error: {:?}", panic);
    }
}