#pragma once

namespace freesthetics
{

    // Enum for status of the program
    enum class Status
    {
        Ok = 1,
        Error,
        RegisterNotUpdated,
        RegisterInvalid,
        OutOfRange,
        Warning
    };

} // namespace freesthetics