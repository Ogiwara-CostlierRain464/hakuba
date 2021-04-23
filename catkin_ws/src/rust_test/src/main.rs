use std::rc::Rc;
use by_address::ByAddress;
use std::borrow::BorrowMut;
use std::cell::RefCell;

const ORDER: usize = 4;

struct Node{
    pointers: [Option<NodeType>; ORDER],
    keys: [isize; ORDER - 1],
    parent: Option<NodeType>,
    is_leaf: bool,
    is_record: bool,
    num_keys: usize
}

type NodeType = Rc<RefCell<Node>>;


fn find_leaf(root: Option<NodeType>, key: isize) -> Option<NodeType>{
    if root.is_none(){
        // empty tree
        return None;
    }
    assert!(root.is_some());
    let mut c = root.unwrap();
    while !c.as_ref().borrow_mut().is_leaf {
        let mut i = 0;
        while i < c.as_ref().borrow_mut().num_keys {
            
        }
    }
    
    return Some(c);
}

fn cut(len: usize) -> usize{
    if len % 2 == 0 {
        len / 2
    }else{
        len / 2 + 1
    }
}


fn make_record() -> NodeType{
    let record = Node{
        pointers: Default::default(),
        keys: Default::default(),
        parent: None,
        is_leaf: false,
        is_record: true,
        num_keys: 0
    };

    Rc::new(RefCell::from(record))
}

// Creates a new general node
fn make_node() -> NodeType{
    let node = Node{
        pointers: Default::default(),
        keys: Default::default(),
        parent: None,
        is_leaf: false,
        is_record: false,
        num_keys: 0
    };

    Rc::new(RefCell::from(node))
}

fn make_leaf() -> NodeType{
    let leaf = make_node();
    leaf.as_ref().borrow_mut().is_leaf = true;
    leaf
}

// Helper function used in insert_into_parent
fn get_left_index(mut parent: NodeType, left: NodeType) -> usize{
    let mut left_index: usize = 0;
    while left_index <= parent.as_ref().borrow_mut().num_keys{
        match &parent.as_ref().borrow_mut().pointers[left_index] {
            Some(x) => if  ByAddress(x) != ByAddress(&left) {
                   left_index += 1;
            },
            _ => {}
        }
    }

    left_index
}

fn insert_into_leaf(mut leaf: NodeType, key: isize, ptr_record: NodeType) -> NodeType{
    let mut insertion_point: usize = 0;

    while insertion_point < leaf.as_ref().borrow_mut().num_keys
        && leaf.as_ref().borrow_mut().keys[insertion_point] < key {
        insertion_point += 1;
    }

    // move right from insertion point.
    for i in (insertion_point+1..leaf.as_ref().borrow_mut().num_keys).rev(){
        leaf.as_ref().borrow_mut().keys[i] = leaf.as_ref().borrow_mut().keys[i -1];
        leaf.as_ref().borrow_mut().pointers[i] = leaf.as_ref().borrow_mut().pointers[i - 1].clone();
    }
    leaf.as_ref().borrow_mut().keys[insertion_point] = key;
    leaf.as_ref().borrow_mut().pointers[insertion_point] = Some(ptr_record);
    leaf.as_ref().borrow_mut().num_keys += 1;

    leaf
}


fn insert_into_node_after_splitting(root: NodeType, mut old_node: NodeType,
                                    left_index: usize, key: isize,
                                    right: NodeType) -> NodeType{
    let mut tmp_pointers: [Option<NodeType>; ORDER+1] = Default::default();
    let mut tmp_keys = [0; ORDER];

    let mut i = 0; let mut j = 0;
    loop {
        if !(i < old_node.as_ref().borrow_mut().num_keys + 1){ break; }
        if j == left_index+ 1{ j+=1; }
        tmp_pointers[j] = old_node.as_ref().borrow_mut().pointers[i].clone();

        i+=1; j+=1;
    }

    i=0;j=0;
    loop {
        if !(i < old_node.as_ref().borrow_mut().num_keys){ break; }
        if j == left_index{ j+=1; }
        tmp_keys[j] = old_node.as_ref().borrow_mut().keys[i];

        i+=1; j+=1;
    }

    tmp_pointers[left_index+1] = Some(right);
    tmp_keys[left_index] = key;

    let split = cut(ORDER);
    let mut new_node = make_node();
    old_node.as_ref().borrow_mut().num_keys = 0;

    i = 0;
    loop {
        if !(i < split - 1){ break }
        old_node.as_ref().borrow_mut().pointers[i] = tmp_pointers[i].clone();
        old_node.as_ref().borrow_mut().keys[i] = tmp_keys[i];
        old_node.as_ref().borrow_mut().num_keys += 1;

        i+=1;
    }
    old_node.as_ref().borrow_mut().pointers[i] = tmp_pointers[i].clone();
    let k_prime = tmp_keys[split - 1];

    j = 0;
    loop{
        if !(i < ORDER){ break; }

        new_node.as_ref().borrow_mut().pointers[j] = tmp_pointers[i].clone();
        new_node.as_ref().borrow_mut().keys[j] = tmp_keys[i];
        new_node.as_ref().borrow_mut().num_keys+=1;

        i+=1; j+=1;
    }
    new_node.as_ref().borrow_mut().pointers[j] = tmp_pointers[i].clone();
    new_node.as_ref().borrow_mut().parent = old_node.as_ref().borrow_mut().parent.clone();

    i=0;
    let mut child;
    loop {
        if !(i <= new_node.as_ref().borrow_mut().num_keys){ break; }
        child = new_node.as_ref().borrow_mut().pointers[i].clone().unwrap();
        child.as_ref().borrow_mut().parent = Some(Rc::clone(&new_node));

        i+=1;
    }

    insert_into_parent(root, old_node, k_prime, new_node)
}


fn insert_into_parent(root: NodeType, left: NodeType,
                      key: isize, right: NodeType) -> NodeType{
    todo!()
}

fn insert_into_leaf_after_splitting(root: NodeType, leaf: NodeType,
                                    key: isize, ptr_record: NodeType) -> NodeType{
 todo!("w")
}


fn main(){
    for i in 1..3{
        println!("{}", i);
    }

}