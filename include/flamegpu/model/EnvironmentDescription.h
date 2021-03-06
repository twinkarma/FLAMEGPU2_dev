#ifndef INCLUDE_FLAMEGPU_MODEL_ENVIRONMENTDESCRIPTION_H_
#define INCLUDE_FLAMEGPU_MODEL_ENVIRONMENTDESCRIPTION_H_

#include <unordered_map>
#include <string>
#include <typeinfo>
#include <typeindex>
#include <array>

#include "flamegpu/exception/FGPUException.h"
#include "flamegpu/runtime/utility/HostEnvironment.cuh"

/**
 * @brief Description class for environment properties
 * 
 * Allows environment properties to be prepared and attached to a ModelDescription.
 * Properties can be any arithmetic or enum type.
 * Properties marked as const within the EnvironmentDescription cannot be changed during the simulation
 */
class EnvironmentDescription {
    /**
     * EnvironmentManager needs access to our internal members
     * @see EnvironmentManager::init(const std::string &, const EnvironmentDescription &)
     */
    friend class EnvironmentManager;
    /**
     * This directly accesses properties map, to build mappings
     * Not required if this class is changed into description/data format like others
     */
    friend class SubEnvironmentDescription;
    /**
     * Minimal std::any replacement, works pre c++17
     * We don't care about type, so it isn't tracked
     */
    struct Any {
        /**
         * Constructor
         * @param _ptr Pointer to data to represent
         * @param _length Length of pointed to data
         * @note Copies the data
         */
        Any(const void *_ptr, const size_t &_length)
            : ptr(malloc(_length)),
            length(_length) {
            memcpy(ptr, _ptr, length);
        }
        /**
         * Copy constructor
         * @param _other Other Any to be cloned, makes a copy of the pointed to data
         */
        Any(const Any &_other)
            : ptr(malloc(_other.length)),
            length(_other.length) {
            memcpy(ptr, _other.ptr, length);
        }
        /*
         * Releases the allocated memory
         */
        ~Any() {
            free(ptr);
        }
        /**
         * Can't assign, members are const at creation
         */
        void operator=(const Any &_other) = delete;
        /**
         * Data represented by this object
         */
        void *const ptr;
        /**
         * Length of memory allocation pointed to by ptr
         */
        const size_t length;
    };

 public:
    /**
     * Holds all of the properties required to add a value to EnvironmentManager
     */
    struct PropData {
        /**
         * @param _is_const Is the property constant
         * @param _elements How many elements does the property have (1 if it's not an array)
         * @param _data The data to initially fill the property with
         * @param _type Identifier of type
         */
        PropData(const bool &_is_const, const EnvironmentManager::size_type &_elements, const Any &_data, const std::type_index &_type)
            : isConst(_is_const),
            elements(_elements),
            data(_data),
            type(_type) { }
        bool isConst;
        const EnvironmentManager::size_type elements;
        const Any data;
        const std::type_index type;
    };
    /**
     * Default destruction
     */
    EnvironmentDescription() = default;

    bool operator==(const EnvironmentDescription& rhs) const;
    bool operator!=(const EnvironmentDescription& rhs) const;
    /**
     * Adds a new environment property
     * @param name name used for accessing the property
     * @param value stored value of the property
     * @param isConst If set to true, it is not possible to change the value during the simulation
     * @tparam T Type of the environmental property to be created
     * @throws DuplicateEnvProperty If a property of the same name already exists
     */
    template<typename T>
    void add(const std::string &name, const T &value, const bool &isConst = false);
    /**
     * Adds a new environment property array
     * @param name Name used for accessing the property
     * @param value Stored value of the property
     * @param isConst If set to true, it is not possible to change the value during the simulation
     * @tparam T Type of the environmental property array to be created
     * @tparam N Length of the environmental property array to be created
     * @throws DuplicateEnvProperty If a property of the same name already exists
     */
    template<typename T, EnvironmentManager::size_type N>
    void add(const std::string &name, const std::array<T, N> &value, const bool &isConst = false);
    /**
     * Gets an environment property
     * @param name name used for accessing the property
     * @tparam T Type of the value to be returned
     * @throws InvalidEnvProperty If a property of the name does not exist
     */
    template<typename T>
    T get(const std::string &name) const;
    /**
     * Gets an environment property array
     * @param name name used for accessing the property
     * @tparam T Type of the value to be returned
     * @tparam N Length of the array to be returned
     * @throws InvalidEnvProperty If a property array of the name does not exist
     */
    template<typename T, EnvironmentManager::size_type N>
    std::array<T, N> get(const std::string &name) const;
    /**
     * Gets an element of an environment property array
     * @param name name used for accessing the property
     * @param index element from the environment property array to return
     * @tparam T Type of the value to be returned
     * @throws InvalidEnvProperty If a property of the name does not exist
     * @throws std::out_of_range
     */
    template<typename T>
    T get(const std::string &name, const EnvironmentManager::size_type &index) const;
    /**
     * Returns whether an environment property is marked as const
     * @param name name used for accessing the property
     * @throws InvalidEnvProperty If a property of the name does not exist
     */
    bool getConst(const std::string &name);
    /**
     * Sets an environment property
     * @param name name used for accessing the property
     * @param value value to set the property
     * @tparam T Type of the value to be returned
     * @return Returns the previous value
     * @throws InvalidEnvProperty If a property of the name does not exist
     */
    template<typename T>
    T set(const std::string &name, const T &value);
    /**
     * Sets an environment property array
     * @param name name used for accessing the property
     * @param value value to set the property
     * @tparam T Type of the value to be returned
     * @tparam N Length of the array to be returned
     * @return Returns the previous value
     * @throws InvalidEnvProperty If a property of the name does not exist
     */
    template<typename T, EnvironmentManager::size_type N>
    std::array<T, N> set(const std::string &name, const std::array<T, N> &value);
    /**
     * Sets an element of an environment property array
     * @param name name used for accessing the property
     * @param index element from the environment property array to set
     * @param value value to set the property
     * @tparam T Type of the value to be returned
     * @return Returns the previous value of the environment property array element which has been set
     * @throws InvalidEnvProperty If a property of the name does not exist
     * @throws std::out_of_range
     * @see set(const std::string &, const T &value)
     */
    template<typename T>
    T set(const std::string &name, const EnvironmentManager::size_type &index, const T &value);

    const std::unordered_map<std::string, PropData> getPropertiesMap() const;

 private:
    /**
     * Internal common add method, actually performs the heavy lifting of changing properties
     * @param name Name used for accessing the property
     * @param ptr Pointer to data to initially fill property with
     * @param len Length of data pointed to by ptr
     * @param isConst If set to true, it is not possible to change the value during the simulation
     * @param elements How many elements does the property have (1 if it's not an array)
     * @param type value returned by typeid()
     */
    void add(const std::string &name, const char *ptr, const size_t &len, const bool &isConst, const EnvironmentManager::size_type &elements, const std::type_index &type);
    /**
     * Main storage of all properties
     */
    std::unordered_map<std::string, PropData> properties;
};


/**
 * Constructors
 */
template<typename T>
void EnvironmentDescription::add(const std::string &name, const T &value, const bool &isConst) {
    if (!name.empty() && name[0] == '_') {
        THROW ReservedName("Environment property names cannot begin with '_', this is reserved for internal usage, "
            "in EnvironmentDescription::add().");
    }
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    if (properties.find(name) != properties.end()) {
        THROW DuplicateEnvProperty("Environmental property with name '%s' already exists, "
            "in EnvironmentDescription::add().",
            name.c_str());
    }
    add(name, reinterpret_cast<const char*>(&value), sizeof(T), isConst, 1, typeid(T));
}
template<typename T, EnvironmentManager::size_type N>
void EnvironmentDescription::add(const std::string &name, const std::array<T, N> &value, const bool &isConst) {
    if (!name.empty() && name[0] == '_') {
        THROW ReservedName("Environment property names cannot begin with '_', this is reserved for internal usage, "
            "in EnvironmentDescription::add().");
    }
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    if (properties.find(name) != properties.end()) {
        THROW DuplicateEnvProperty("Environmental property with name '%s' already exists, "
            "in EnvironmentDescription::add().",
            name.c_str());
    }
    add(name, reinterpret_cast<const char*>(value.data()), N * sizeof(T), isConst, N, typeid(T));
}

/**
 * Getters
 */
template<typename T>
T EnvironmentDescription::get(const std::string &name) const {
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::get().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        return *reinterpret_cast<T*>(i->second.data.ptr);
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::get().",
        name.c_str());
}
template<typename T, EnvironmentManager::size_type N>
std::array<T, N> EnvironmentDescription::get(const std::string &name) const {
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property array ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::get().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        if (i->second.elements != N) {
            THROW OutOfBoundsException("Length of named environmental property array (%u) does not match template argument N (%u), "
                "in EnvironmentDescription::get().",
                i->second.elements, N);
        }
        // Copy old data to return
        std::array<T, N> rtn;
        memcpy(rtn.data(), reinterpret_cast<T*>(i->second.data.ptr), N * sizeof(T));
        return rtn;
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::get().",
        name.c_str());
}
template<typename T>
T EnvironmentDescription::get(const std::string &name, const EnvironmentManager::size_type &index) const {
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property array ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::get().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        if (i->second.elements <= index) {
            THROW OutOfBoundsException("Index (%u) exceeds named environmental property array's length (%u), "
                "in EnvironmentDescription::get().",
                index, i->second.elements);
        }
        // Copy old data to return
        return *(reinterpret_cast<T*>(i->second.data.ptr) + index);
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::get().",
        name.c_str());
}

/**
 * Setters
 */
template<typename T>
T EnvironmentDescription::set(const std::string &name, const T &value) {
    if (!name.empty() && name[0] == '_') {
        THROW ReservedName("Environment property names cannot begin with '_', this is reserved for internal usage, "
            "in EnvironmentDescription::set().");
    }
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::set().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        // Copy old data to return
        T rtn = *reinterpret_cast<T*>(i->second.data.ptr);
        // Store data
        memcpy(i->second.data.ptr, &value, sizeof(T));
        return rtn;
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::set().",
        name.c_str());
}
template<typename T, EnvironmentManager::size_type N>
std::array<T, N> EnvironmentDescription::set(const std::string &name, const std::array<T, N> &value) {
    if (!name.empty() && name[0] == '_') {
        THROW ReservedName("Environment property names cannot begin with '_', this is reserved for internal usage, "
            "in EnvironmentDescription::set().");
    }
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property array ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::set().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        if (i->second.elements != N) {
            THROW OutOfBoundsException("Length of named environmental property array (%u) does not match template argument N (%u), "
                "in EnvironmentDescription::set().",
                i->second.elements, N);
        }
        // Copy old data to return
        std::array<T, N> rtn;
        memcpy(rtn.data(), reinterpret_cast<T*>(i->second.data.ptr), N * sizeof(T));
        // Store data
        memcpy(reinterpret_cast<T*>(i->second.data.ptr), &value, N * sizeof(T));
        return rtn;
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::set().",
        name.c_str());
}
template<typename T>
T EnvironmentDescription::set(const std::string &name, const EnvironmentManager::size_type &index, const T &value) {
    if (!name.empty() && name[0] == '_') {
        THROW ReservedName("Environment property names cannot begin with '_', this is reserved for internal usage, "
            "in EnvironmentDescription::set().");
    }
    // Limited to Arithmetic types
    // Compound types would allow host pointers inside structs to be passed
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
        "Only arithmetic types can be used as environmental properties");
    auto &&i = properties.find(name);
    if (i != properties.end()) {
        if (i->second.type != std::type_index(typeid(T))) {
            THROW InvalidEnvPropertyType("Environmental property array ('%s') type (%s) does not match template argument T (%s), "
                "in EnvironmentDescription::set().",
                name.c_str(), i->second.type.name(), typeid(T).name());
        }
        if (i->second.elements <= index) {
            THROW OutOfBoundsException("Index (%u) exceeds named environmental property array's length (%u), "
                "in EnvironmentDescription::set().",
                index, i->second.elements);
        }
        // Copy old data to return
        T rtn = *(reinterpret_cast<T*>(i->second.data.ptr) + index);
        // Store data
        memcpy(reinterpret_cast<T*>(i->second.data.ptr) + index, &value, sizeof(T));
        return rtn;
    }
    THROW InvalidEnvProperty("Environmental property with name '%s' does not exist, "
        "in EnvironmentDescription::set().",
        name.c_str());
}
#endif  // INCLUDE_FLAMEGPU_MODEL_ENVIRONMENTDESCRIPTION_H_
