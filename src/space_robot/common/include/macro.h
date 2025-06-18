#ifndef COMMON_MACRO_H
#define COMMON_MACRO_H

#pragma once
#include <memory>


#define DECLARE_PTR(Name, Type)                             \
    using Name##Ptr = std::shared_ptr<Type>;                \
    using Name##ConstPtr = std::shared_ptr<const Type>;     \
    using Name##WeakPtr = std::weak_ptr<Type>;              \
    using Name##ConstWeakPtr = std::weak_ptr<const Type>;   \
    using Name##UniquePtr = std::unique_ptr<Type>;          \
    using Name##UniqueConstPtr = std::unique_ptr<const Type>


#define DECLARE_PTR_MEMBER(Type)                            \
    using Ptr = std::shared_ptr<Type>;                      \
    using ConstPtr = std::shared_ptr<const Type>;           \
    using WeakPtr = std::weak_ptr<Type>;                    \
    using ConstWeakPtr = std::weak_ptr<const Type>;         \
    using UniquePtr = std::unique_ptr<Type>;                \
    using UniqueConstPtr = std::unique_ptr<const Type>


#define CLASS_FORWARD(C)    \
    class C;                \
    DECLARE_PTR(C, C)


#define STRUCT_FORWARD(C)   \
    struct C;               \
    DECLARE_PTR(C, C)


#endif