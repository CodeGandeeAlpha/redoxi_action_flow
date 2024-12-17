#pragma once

// basic concepts
#include <chrono>
#include <concepts>
#include <redoxi_basic_cpp/types/rdx_uuid.hpp>
#include <redoxi_basic_cpp/types/rdx_enums.hpp>

namespace redoxi_works
{
//! Concept to check if a type is std::chrono::duration
template <typename T>
concept TimeDurationConcept = requires
{
    requires std::is_same_v<T, std::chrono::duration<typename T::rep, typename T::period>>;
};

// hacky stuff
namespace hacky
{

/**
 * @brief Type trait to check if a type is a std::chrono::duration
 * @details Usage:
 * template <typename T, typename = std::enable_if_t<is_duration<T>::value>>
 * void function(T duration) { ... }
 *
 * Or in C++20:
 * template <typename T>
 *     requires is_duration<T>::value
 * void function(T duration) { ... }
 */
template <typename T, typename = void>
struct is_duration : std::false_type {
};

/**
 * @brief Specialization for types that satisfy the requirements of std::chrono::duration
 * @tparam T The type to check
 * @details This trait is true for types that have a count() method, a rep type,
 *          a period type, and are convertible to std::chrono::duration
 */
template <typename T>
struct is_duration<T, std::void_t<decltype(std::declval<T>().count()),
                                  typename T::rep,
                                  typename T::period>> : std::is_convertible<T, std::chrono::duration<typename T::rep,
                                                                                                      typename T::period>> {
};

} // namespace hacky

} // namespace redoxi_works
