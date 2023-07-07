#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(DataStreamOut) {
#define PYB_CUR_CLASS DataStreamOut
	PYB_CLASS(DataStreamOut)
		PYB_DEFAULT_INITS()

		.def("get_pos", [](const DataStreamOut& out) { return out.pos; })

		.def("to_byte_array",
			[](const DataStreamOut& out) {
				return pyb::cast(out.data);
			}
		)

		.def("write_version_check", &DataStreamOut::WriteVersionCheck)
		.def("write_to_file", 
			[](DataStreamOut& out, wstring path, bool writeVersionCheck) { 
				out.WriteToFile(path, writeVersionCheck); 
			},
			PYBA("file_path"), PYBA("write_version_check") = true
		)
		;
}
#endif