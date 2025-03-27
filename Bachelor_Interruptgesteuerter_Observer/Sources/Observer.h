/*
 * Observer.h
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

// Struct to combine key and function pointer
typedef struct {
    const char* key;    // String key for the function
    void* func;         // Pointer to the function
} ObserverFuncEntry;

// Dictionary of function pointers with their keys
static ObserverFuncEntry OBSERVER_FUNC_DICT[] = {
    {"rdm", (void*)read_mem},     // read_mem function
    {"wrm", (void*)write_mem},    // write_mem function
    {"inr", (void*)interrupt}     // interrupt function
};

// Number of functions in the dictionary
#define OBSERVER_FUNC_COUNT (sizeof(OBSERVER_FUNC_DICT) / sizeof(OBSERVER_FUNC_DICT[0]))

// Function declarations
EXTERN void Observer_init(void);
EXTERN int Observer_print(const char * str);

EXTERN int read_mem(const int str, const int addr, const int blocks);
EXTERN int write_mem(const int str, const int addr, const int blocks);
EXTERN int interrupt();


#endif /* OBSERVER_H_ */
