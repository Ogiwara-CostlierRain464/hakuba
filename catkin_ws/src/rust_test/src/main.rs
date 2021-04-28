use std::rc::Rc;
use by_address::ByAddress;
use std::cell::RefCell;

const ORDER: usize = 4;

struct Node{
    pointers: [Option<NodeType>; ORDER],
    keys: [isize; ORDER - 1],
    parent: Option<NodeType>,
    is_leaf: bool,
    is_record: bool,
    value: i128,
    num_keys: usize,
}

type NodeType = Rc<RefCell<Node>>;


fn find_leaf(root: Option<NodeType>, key: isize) -> Option<NodeType>{
    if root.is_none(){
        // empty tree
        return None;
    }
    assert!(root.is_some());
    let mut c = root.unwrap();
    while !c.as_ref().borrow().is_leaf {
        let mut i = 0;
        while i < c.as_ref().borrow().num_keys {
            if key >= c.as_ref().borrow().keys[i] {  i+=1; }
            else { break; }
        }
        c = c.clone().as_ref().borrow().pointers[i].clone().unwrap();
    }
    
    return Some(c);
}

fn find(root: Option<NodeType>, key: isize) -> Option<NodeType>{
    if root.is_none(){
        return None;
    }

    let leaf = find_leaf(root, key);
    let mut i = 0;
    assert!(leaf.is_some());
    while i < leaf.as_ref().unwrap().as_ref().borrow_mut().num_keys {
        if leaf.as_ref().unwrap().as_ref().borrow_mut().keys[i] == key{ break; }
        i+=1;
    }

    return if i == leaf.as_ref().unwrap().as_ref().borrow_mut().num_keys {
        None
    } else {
        leaf.unwrap().as_ref().borrow_mut().pointers[i].clone()
    }
}

fn cut(len: usize) -> usize{
    if len % 2 == 0 {
        len / 2
    }else{
        len / 2 + 1
    }
}


fn make_record(value: i128) -> NodeType{
    let record = Node{
        pointers: Default::default(),
        keys: Default::default(),
        parent: None,
        is_leaf: false,
        is_record: true,
        value,
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
        value: 0,
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
fn get_left_index(parent: NodeType, left: NodeType) -> usize{
    let mut left_index: usize = 0;
    while left_index <= parent.as_ref().borrow().num_keys
        && ByAddress(parent.as_ref().borrow().pointers[left_index].as_ref().unwrap().as_ref().borrow())
        != ByAddress(left.as_ref().borrow()){
        left_index += 1;
    }


    left_index
}

fn insert_into_leaf(leaf: NodeType, key: isize, ptr_record: NodeType) -> NodeType{
    let mut leaf_ref = leaf.as_ref().borrow_mut();
    let mut insertion_point: usize = 0;

    while insertion_point < leaf_ref.num_keys
        && leaf_ref.keys[insertion_point] < key {
        insertion_point += 1;
    }

    let mut i = leaf_ref.num_keys;
    while i > insertion_point{
        leaf_ref.keys[i] = leaf_ref.keys[i-1];
        leaf_ref.pointers[i] = leaf_ref.pointers[i-1].clone();
        i-=1;
    }

    leaf_ref.keys[insertion_point] = key;
    leaf_ref.pointers[insertion_point] = Some(ptr_record);
    leaf_ref.num_keys += 1;

    return leaf.clone()
}

fn insert_into_node(root: NodeType, n: NodeType,
                    left_index: usize, key: isize, right: NodeType)
-> NodeType{
    let mut i: usize = n.as_ref().borrow_mut().num_keys;

    loop{
        if !(i > left_index){ break; }
        n.as_ref().borrow_mut().pointers[i+1] = n.as_ref().borrow_mut().pointers[i].clone();
        n.as_ref().borrow_mut().keys[i] = n.as_ref().borrow_mut().keys[i - 1];
        i-=1;
    }
    n.as_ref().borrow_mut().pointers[left_index + 1] = Some(right);
    n.as_ref().borrow_mut().keys[left_index] = key;
    n.as_ref().borrow_mut().num_keys += 1;
    return root;
}

fn insert_into_node_after_splitting(root: NodeType, old_node: NodeType,
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
    let new_node = make_node();
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
    let left_index: usize;
    let parent;

    parent = left.as_ref().borrow().parent.clone();

    if parent.is_none(){
        return insert_into_new_root(left, key, right);
    }
    assert!(parent.is_some());

    // Find the parent's pointer to the left node
    left_index = get_left_index(parent.clone().unwrap(), left);

    if parent.as_ref().unwrap()
        .as_ref()
        .borrow_mut().num_keys < ORDER - 1{
        return insert_into_node(root, parent.unwrap(), left_index, key, right);
    }

    return insert_into_node_after_splitting(root, parent.unwrap(), left_index, key, right);
}

fn insert_into_new_root(left: NodeType, key: isize, right: NodeType)
    -> NodeType{
    let root = make_node();
    root.as_ref().borrow_mut().keys[0] = key;
    root.as_ref().borrow_mut().pointers[0] = Some(left.clone());
    root.as_ref().borrow_mut().pointers[1] = Some(right.clone());
    root.as_ref().borrow_mut().num_keys += 1;
    root.as_ref().borrow_mut().parent = None;
    left.as_ref().borrow_mut().parent = Some(root.clone());
    right.as_ref().borrow_mut().parent = Some(root.clone());

    return root;
}

fn insert_into_leaf_after_splitting(root: NodeType, leaf: NodeType,
                                    key: isize, ptr_record: NodeType)
-> NodeType{
    let new_leaf = make_leaf();
    let mut tmp_keys = [0; ORDER];
    let mut tmp_pointers: [Option<NodeType>; ORDER+1] = Default::default();

    let mut insertion_index = 0;
    while insertion_index < ORDER - 1
        && leaf.as_ref().borrow_mut().keys[insertion_index] < key {
        insertion_index+=1;
    }

    let mut i = 0; let mut j = 0;
    loop{
        if !(i < leaf.as_ref().borrow_mut().num_keys){ break; }

        if j == insertion_index{ j+=1; }
        tmp_keys[j] = leaf.as_ref().borrow_mut().keys[i];
        tmp_pointers[j] = leaf.as_ref().borrow_mut().pointers[i].clone();

        i+=1; j+=1;
    }

    tmp_keys[insertion_index] = key;
    tmp_pointers[insertion_index] = Some(ptr_record);

    leaf.as_ref().borrow_mut().num_keys = 0;

    let split = cut(ORDER - 1);

    i=0;
    while i < split {
        leaf.as_ref().borrow_mut().pointers[i] = tmp_pointers[i].clone();
        leaf.as_ref().borrow_mut().keys[i] = tmp_keys[i];
        leaf.as_ref().borrow_mut().num_keys +=1;

        i +=1;
    }

    i = split; j=0;
    while i < ORDER {
        new_leaf.as_ref().borrow_mut().pointers[j] = tmp_pointers[i].clone();
        new_leaf.as_ref().borrow_mut().keys[j] = tmp_keys[i];
        new_leaf.as_ref().borrow_mut().num_keys +=1;

        i+=1; j+=1;
    }

    new_leaf.as_ref().borrow_mut().pointers[ORDER-1]
        = leaf.as_ref().borrow_mut().pointers[ORDER-1].clone();

    leaf.as_ref().borrow_mut().pointers[ORDER-1] = Some(new_leaf.clone());

    i=leaf.as_ref().borrow_mut().num_keys;
    while i < ORDER-1 {
        leaf.as_ref().borrow_mut().pointers[i] = None;
        i+=1;
    }
    i=new_leaf.as_ref().borrow_mut().num_keys;
    while i < ORDER-1 {
        new_leaf.as_ref().borrow_mut().pointers[i] = None;
        i+=1;
    }

    new_leaf.as_ref().borrow_mut().parent = leaf.as_ref().borrow_mut().parent.clone();
    let new_key = new_leaf.as_ref().borrow_mut().keys[0];

    return insert_into_parent(root, leaf, new_key, new_leaf);
}

fn start_new_tree(key: isize, record_ptr: NodeType)
-> NodeType{
    let root = make_leaf();
    root.as_ref().borrow_mut().keys[0] = key;
    root.as_ref().borrow_mut().pointers[0] = Some(record_ptr);
    root.as_ref().borrow_mut().pointers[ORDER-1] = None;
    root.as_ref().borrow_mut().parent = None;
    root.as_ref().borrow_mut().num_keys +=1;
    return root;
}

fn insert(root: Option<NodeType>, key: isize, value: i128) -> NodeType{
    let mut record_ptr = find(root.clone(), key);
    if record_ptr.is_some(){
        // update
        record_ptr.unwrap().as_ref().borrow_mut().value = value;
        return root.unwrap();
    }

    record_ptr = Some(make_record(value));

    if root.as_ref().is_none(){
        return start_new_tree(key, record_ptr.unwrap());
    }

    let mut leaf = find_leaf(root.clone(), key);
    assert!(leaf.as_ref().is_some());

    if leaf.as_ref().unwrap().as_ref().borrow().num_keys < ORDER-1{
        leaf = Some(insert_into_leaf(leaf.unwrap(), key, record_ptr.unwrap()));
        return root.unwrap();
    }

    return insert_into_leaf_after_splitting(root.unwrap(), leaf.unwrap(), key, record_ptr.unwrap());
}


fn main(){
    let mut root = insert(None, 10, 10);
    root = insert(Some(root), 9, 9);
    root = insert(Some(root), 8, 8);
    root = insert(Some(root), 7, 7);
    root = insert(Some(root), 11, 11);
    root = insert(Some(root), 12, 12);
    root = insert(Some(root), 13, 13);
    root = insert(Some(root), 14, 14);
    root = insert(Some(root), 15, 15);
    root = insert(Some(root), 16, 16);



    let record = find(Some(root), 16);
    assert!(record.is_some() && record.unwrap().as_ref().borrow().value == 16);
}