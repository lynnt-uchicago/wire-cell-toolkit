/**
   Utilities support the tensor data model common API.
 */
#ifndef WIRECELLAUX_TENSORDMCOMMON
#define WIRECELLAUX_TENSORDMCOMMON

#include "WireCellIface/ITensorSet.h"



namespace WireCell::Aux::TensorDM {

    /** Build metadata-only (array-less) tensor in the DM.
     */
    ITensor::pointer make_metadata_tensor(const std::string& datatype,
                                          const std::string& datapath,
                                          Configuration metadata = {});

    // Helper to access a set of ITensors
    class TensorIndex {
        ITensor::vector m_tens;
        std::unordered_map<std::string, size_t> m_path2ind;
        std::unordered_map<std::string, size_t> m_type2ind;

      public:
        TensorIndex();

        // Construct with a set of tensors to index
        explicit TensorIndex(const ITensor::vector& tens);

        // Add more tensors to the index
        void add(const ITensor::vector& tens);

        // Add one tensor to the index
        void add(const ITensor::pointer& ten);

        // Return the first tensor in list with matching datatype or throw KeyError.
        ITensor::pointer at_of(const std::string& datatype) const;

        // Return the tensor at datapath or throw KeyError.
        ITensor::pointer at(const std::string& datapath) const;

        // Return the tensor of datatype at datapath or throw KeyError.  If
        // datapath is empty, then act as at_of().  Throws KeyError if lookup fails.
        ITensor::pointer at(const std::string& datapath, const std::string& datatype) const;

        // Return the first tensor in list with matching datatype or return def.
        ITensor::pointer get_of(const std::string& datatype, ITensor::pointer def = nullptr) const;

        // Get tensor at datapath or return default.
        ITensor::pointer get(const std::string& datapath, ITensor::pointer def = nullptr) const;

        // Get tensor of datatype at datapath or return def.  If datapath is
        // empty act like get_of().
        ITensor::pointer get(const std::string& datapath, const std::string& datatype,
                             ITensor::pointer def = nullptr) const;

    };


    /// Build a tensor set from set of tensors.  The tensor data model
    /// makes no requirements on the ITensorSet itself.  The user may
    /// combine tensors representing multiple objects into one set.
    ITensorSet::pointer as_tensorset(const ITensor::vector& tens,
                                     int ident = 0,
                                     const Configuration& tsetmd = Json::objectValue);


}

#endif
