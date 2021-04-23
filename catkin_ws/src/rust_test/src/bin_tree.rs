use std::boxed::Box;

#[derive(Debug)]
pub struct Node{
    data: isize,
    left: Option<Box<Node>>,
    right: Option<Box<Node>>,
}

impl Node{
    pub(crate) fn new(num: isize) -> Self{
        Self {
            data: num,
            left: None,
            right: None,
        }
    }

    pub(crate) fn insert(&mut self, num: isize){
        if self.data > num {
            if let Some(left_node) = &mut self.left {
                left_node.insert(num);
            } else {
                let node = Self::new(num);
                self.left = Some(Box::new(node));
            }
        } else if self.data < num {
            if let Some(right_node) = &mut self.right {
                right_node.insert(num);
            } else {
                let node = Self::new(num);
                self.right = Some(Box::new(node));
            }
        } else {
            assert_eq!(self.data, num);
        }
    }

    pub(crate) fn search(&self, num: isize) -> bool {
        if self.data > num {
            if let Some(left_node) = &self.left {
                left_node.search(num)
            }else { false }
        } else if self.data < num {
            if let Some(right_node) = &self.right {
                right_node.search(num)
            }else { false }
        } else { true }
    }
}
