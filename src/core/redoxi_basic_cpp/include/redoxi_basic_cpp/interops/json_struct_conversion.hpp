#pragma once

#include <json_struct/json_struct.h>
#include <redoxi_basic_cpp/types/rdx_enums.hpp>
#include <redoxi_basic_cpp/concepts/basic_concepts.hpp>

namespace JS
{
//! convert enum to and from string

//! Concept to check if a type is an enum string mapper
template <typename T>
concept EnumStringMapperConcept = requires(T e)
{
    typename T::EnumType_t;
    requires std::is_enum_v<typename T::EnumType_t>;

    //! Get mapping from string to enum value
    {
        T::get_str_to_enum()
        } -> std::same_as<const std::map<std::string, typename T::EnumType_t> &>;

    //! Get mapping from enum value to string
    {
        T::get_enum_to_str()
        } -> std::same_as<const std::map<typename T::EnumType_t, std::string> &>;
};

template <EnumStringMapperConcept Mapper>
struct TypeHandlerWithMapper {
    using EnumType_t = typename Mapper::EnumType_t;
    using Mapper_t = Mapper;

    static inline Error to(EnumType_t &to_type, ParseContext &context)
    {
        //! First try to parse as string
        const auto &str_to_enum = Mapper::get_str_to_enum();
        std::string str;
        Error err = TypeHandler<std::string>::to(str, context);

        if (err == Error::NoError) {
            //! Convert to lower case for case-insensitive comparison
            std::transform(str.begin(), str.end(), str.begin(), ::tolower);

            auto it = str_to_enum.find(str);
            if (it != str_to_enum.end()) {
                to_type = it->second;
                return Error::NoError;
            }
            return Error::IllegalDataValue;
        }
        return err;
    }

    static inline void from(const EnumType_t &from_type, Token &token, Serializer &serializer)
    {
        const auto &enum_to_str = Mapper::get_enum_to_str();

        //! Convert to string using enum_to_str map
        auto it = enum_to_str.find(from_type);
        if (it != enum_to_str.end()) {
            TypeHandler<std::string>::from(it->second, token, serializer);
        }
    }
};
} // namespace JS

namespace JS
{
//! Type handler for time duration
template <redoxi_works::TimeDurationConcept T>
struct TypeHandler<T> {
    static Error to(T &to_type, ParseContext &context)
    {
        typename T::rep value;
        auto err = TypeHandler<typename T::rep>::to(value, context);
        if (err != Error::NoError)
            return err;
        to_type = T(value);
        return Error::NoError;
    }
    static void from(const T &from_type, Token &token, Serializer &serializer)
    {
        typename T::rep value = from_type.count();
        TypeHandler<typename T::rep>::from(value, token, serializer);
    }
};
} // namespace JS

namespace JS
{
struct ControlSignalCodeMapper {
    using EnumType_t = redoxi_works::ControlSignalCode;
    using Mapper_t = ControlSignalCodeMapper;
    ControlSignalCodeMapper()
    {
        static_assert(EnumStringMapperConcept<ControlSignalCodeMapper>, "ControlSignalCodeMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"normal", EnumType_t::Normal},
        {"ping", EnumType_t::Ping},
        {"flush", EnumType_t::Flush},
        {"reset", EnumType_t::Reset},
        {"terminate", EnumType_t::Terminate},
    };

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::Normal, "normal"},
        {EnumType_t::Ping, "ping"},
        {EnumType_t::Flush, "flush"},
        {EnumType_t::Reset, "reset"},
        {EnumType_t::Terminate, "terminate"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};


template <>
struct TypeHandler<redoxi_works::ControlSignalCode> : public TypeHandlerWithMapper<ControlSignalCodeMapper> {
};

struct DeliveryPreconditionMapper {
    using EnumType_t = redoxi_works::DeliveryPrecondition;
    using Mapper_t = DeliveryPreconditionMapper;
    DeliveryPreconditionMapper()
    {
        static_assert(EnumStringMapperConcept<DeliveryPreconditionMapper>, "DeliveryPreconditionMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"dont_care", EnumType_t::DontCare},
        {"no_precondition", EnumType_t::NoPrecondition},
        {"any_downstream_ready", EnumType_t::AnyDownstreamReady},
        {"all_downstreams_ready", EnumType_t::AllDownstreamsReady},
        {"custom_1", EnumType_t::Custom_1},
        {"custom_2", EnumType_t::Custom_2},
        {"custom_3", EnumType_t::Custom_3},
        {"custom_4", EnumType_t::Custom_4},
        {"custom_5", EnumType_t::Custom_5},
        {"custom_6", EnumType_t::Custom_6},
        {"custom_7", EnumType_t::Custom_7},
        {"custom_8", EnumType_t::Custom_8},
        {"custom_9", EnumType_t::Custom_9}};

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::DontCare, "dont_care"},
        {EnumType_t::NoPrecondition, "no_precondition"},
        {EnumType_t::AnyDownstreamReady, "any_downstream_ready"},
        {EnumType_t::AllDownstreamsReady, "all_downstreams_ready"},
        {EnumType_t::Custom_1, "custom_1"},
        {EnumType_t::Custom_2, "custom_2"},
        {EnumType_t::Custom_3, "custom_3"},
        {EnumType_t::Custom_4, "custom_4"},
        {EnumType_t::Custom_5, "custom_5"},
        {EnumType_t::Custom_6, "custom_6"},
        {EnumType_t::Custom_7, "custom_7"},
        {EnumType_t::Custom_8, "custom_8"},
        {EnumType_t::Custom_9, "custom_9"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};


template <>
struct TypeHandler<redoxi_works::DeliveryPrecondition> : public TypeHandlerWithMapper<DeliveryPreconditionMapper> {
};

//! String mapper for DropStrategy enum
struct DropStrategyMapper {
    using EnumType_t = redoxi_works::DropStrategy;

    static inline void validate()
    {
        static_assert(EnumStringMapperConcept<DropStrategyMapper>, "DropStrategyMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"dont_care", EnumType_t::DontCare},
        {"no_drop", EnumType_t::NoDrop},
        {"drop_as_needed", EnumType_t::DropAsNeeded},
        {"custom_1", EnumType_t::Custom_1},
        {"custom_2", EnumType_t::Custom_2},
        {"custom_3", EnumType_t::Custom_3},
        {"custom_4", EnumType_t::Custom_4},
        {"custom_5", EnumType_t::Custom_5},
        {"custom_6", EnumType_t::Custom_6},
        {"custom_7", EnumType_t::Custom_7},
        {"custom_8", EnumType_t::Custom_8},
        {"custom_9", EnumType_t::Custom_9}};

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::DontCare, "dont_care"},
        {EnumType_t::NoDrop, "no_drop"},
        {EnumType_t::DropAsNeeded, "drop_as_needed"},
        {EnumType_t::Custom_1, "custom_1"},
        {EnumType_t::Custom_2, "custom_2"},
        {EnumType_t::Custom_3, "custom_3"},
        {EnumType_t::Custom_4, "custom_4"},
        {EnumType_t::Custom_5, "custom_5"},
        {EnumType_t::Custom_6, "custom_6"},
        {EnumType_t::Custom_7, "custom_7"},
        {EnumType_t::Custom_8, "custom_8"},
        {EnumType_t::Custom_9, "custom_9"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};

template <>
struct TypeHandler<redoxi_works::DropStrategy> : public TypeHandlerWithMapper<DropStrategyMapper> {
};


} // namespace JS

namespace redoxi_works
{
inline const std::string &drop_strategy_to_string(DropStrategy drop_strategy)
{
    return JS::DropStrategyMapper::get_enum_to_str().at(drop_strategy);
}

inline const std::string &precondition_to_string(DeliveryPrecondition precondition)
{
    return JS::DeliveryPreconditionMapper::get_enum_to_str().at(precondition);
}
} // namespace redoxi_works
