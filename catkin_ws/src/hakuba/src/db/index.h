#ifndef HAKUBA_INDEX_H
#define HAKUBA_INDEX_H

#include "buffer.h"

// Index represented as linear list.
// each data are retrieved by seeking each page
// how to utilize page system?
// to make structure simple,
// Root is working as meta
// each node points to next node
// In KOBA789's impl, LayoutVerified is used.
// Just an easiest way to convert from struct to array is use C++ casting
// However, I'm not sure this works in another platform.
/**
 * KOBA789's implmentation is mixing domain entity and I/O
 * how can we separate this?
 * Also, KOBA789's impl can hold only one table!
 *
 * you need: Slotted, Pointer, Pointers, LayoutAligned.
 *
 * As same as B+tree node, LinearListNode have to handle Slotted.
 * Size of slot is depend on inserted element
 *
 * Also, in KOBA789's impl, you have to remember Page ID by your self.
 *
 * But anyway, at first, I impl this by single table.
 *
 * How to apply changes to buffer page?
 * Table insert -> Insert to last node(check doubling?)
 * If Last node has capacity, then just insert.
 * Else, create new node and insert to it.
 * Table handle's DiskBufferManager.
 * However, Linear List Node only handle's Buffer, which don't
 * handle I/O by itself.
 */

struct LinearListNode{
  // you need header?
  // node::Node::Header : Only Node type
  // node::Leaf::Header : next_page_id
  // slotted::Header : num_slots, free_space_offset, _pad(padding?)
  // Node[{Node header}Leaf[{Leaf header} Slotted[{Slotted header} body(array) ]]]


  PageId nextPageId;

  // node itself don't have any method!
  // it is just a container
  // how to handle search algorithm?
  void try_insert(/* key and value, write into Buffer */){
    // Handle it's capacity by itself.
    // if capacity is over, notify insert has failed.

  }
};

struct LinearListIndex{

};


#endif //HAKUBA_INDEX_H
