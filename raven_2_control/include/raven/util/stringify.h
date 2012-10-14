/*
 * stringify.h
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#ifndef STRINGIFY_H_
#define STRINGIFY_H_


#ifndef STRINGIFY_VA_ARGS
#define STRINGIFY_VA_ARGS(...) STRINGIFY_VA_ARGS_HELPER(...)
#define STRINGIFY_VA_ARGS_HELPER(...) #__VA_ARGS__
#endif

#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s
#endif

#endif /* STRINGIFY_H_ */
