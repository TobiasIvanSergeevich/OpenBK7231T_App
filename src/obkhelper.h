#ifndef __OBKHELPER_H__
#define __OBKHELPER_H__

//#include "obkdef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@{*/

/**
 * obk_container_of - return the start address of struct type, while ptr is the
 * member of struct type.
 */
#define obk_container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))


/**
 * @brief initialize a list object
 */
#define OBK_LIST_OBJECT_INIT(object) { &(object), &(object) }

/**
 * @brief initialize a list
 *
 * @param l list to be initialized
 */
inline void obk_list_init(obk_list_t *l)
{
    l->next = l->prev = l;
}

/**
 * @brief insert a node after a list
 *
 * @param l list to insert it
 * @param n new node to be inserted
 */
inline void obk_list_insert_after(obk_list_t *l, obk_list_t *n)
{
    l->next->prev = n;
    n->next = l->next;

    l->next = n;
    n->prev = l;
}

/**
 * @brief insert a node before a list
 *
 * @param n new node to be inserted
 * @param l list to insert it
 */
inline void obk_list_insert_before(obk_list_t *l, obk_list_t *n)
{
    l->prev->next = n;
    n->prev = l->prev;

    l->prev = n;
    n->next = l;
}

/**
 * @brief remove node from list.
 * @param n the node to remove from the list.
 */
inline void obk_list_remove(obk_list_t *n)
{
    n->next->prev = n->prev;
    n->prev->next = n->next;

    n->next = n->prev = n;
}

/**
 * @brief tests whether a list is empty
 * @param l the list to test.
 */
inline int obk_list_isempty(const obk_list_t *l)
{
    return l->next == l;
}

/**
 * @brief get the list length
 * @param l the list to get.
 */
inline unsigned int obk_list_len(const obk_list_t *l)
{
    unsigned int len = 0;
    const obk_list_t *p = l;
    while (p->next != l)
    {
        p = p->next;
        len ++;
    }

    return len;
}

/**
 * @brief get the struct for this entry
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define obk_list_entry(node, type, member) \
    obk_container_of(node, type, member)

/**
 * obk_list_for_each - iterate over a list
 * @param pos the obk_list_t * to use as a loop cursor.
 * @param head the head for your list.
 */
#define obk_list_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * obk_list_for_each_safe - iterate over a list safe against removal of list entry
 * @param pos the obk_list_t * to use as a loop cursor.
 * @param n another obk_list_t * to use as temporary storage
 * @param head the head for your list.
 */
#define obk_list_for_each_safe(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

/**
 * obk_list_for_each_entry  -   iterate over list of given type
 * @param pos the type * to use as a loop cursor.
 * @param head the head for your list.
 * @param member the name of the list_struct within the struct.
 */
#define obk_list_for_each_entry(pos, head, member) \
    for (pos = obk_list_entry((head)->next, obk_typeof(*pos), member); \
         &pos->member != (head); \
         pos = obk_list_entry(pos->member.next, obk_typeof(*pos), member))

/**
 * obk_list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @param pos the type * to use as a loop cursor.
 * @param n another type * to use as temporary storage
 * @param head the head for your list.
 * @param member the name of the list_struct within the struct.
 */
#define obk_list_for_each_entry_safe(pos, n, head, member) \
    for (pos = obk_list_entry((head)->next, obk_typeof(*pos), member), \
         n = obk_list_entry(pos->member.next, obk_typeof(*pos), member); \
         &pos->member != (head); \
         pos = n, n = obk_list_entry(n->member.next, obk_typeof(*n), member))

/**
 * obk_list_first_entry - get the first element from a list
 * @param ptr the list head to take the element from.
 * @param type the type of the struct this is embedded in.
 * @param member the name of the list_struct within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define obk_list_first_entry(ptr, type, member) \
    obk_list_entry((ptr)->next, type, member)

#endif //__OBKHELPER_H__