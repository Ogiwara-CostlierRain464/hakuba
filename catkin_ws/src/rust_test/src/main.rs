use std::cmp::{min, max};

pub fn ants (l: i32, ants: &[i32]){
    let mut min_t = 0;
    for &ant_pos in ants{
        min_t = max(min_t, min(ant_pos, l - ant_pos));
    }

    let mut max_t = 0;
    for &ant_pos in ants{
        max_t = max(max_t, max(ant_pos, l - ant_pos));
    }

    println!("min: {} max: {}", min_t, max_t);
}

fn main() {
    ants(10, &[1,3, 5, 6, 7,2,4]);
}
