#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(DataStreamIn) {
#define PYB_CUR_CLASS DataStreamIn
	PYB_CLASS(DataStreamIn)
		PYB_DEFAULT_INITS()

		.def("get_pos", [](const DataStreamIn& in) { return in.pos; })
		.def("is_done", [](const DataStreamIn& in) { return in.IsDone(); })
		.def("is_overflown", [](const DataStreamIn& in) { return in.IsOverflown(); })

		.def("to_byte_array", 
			[](const DataStreamIn& in) {
				return pyb::cast(in.data);
			}
		)

		.def("read_version_check", &DataStreamIn::DoVersionCheck)

		.def_static("read_from_file",
			[](wstring path, bool readVersionCheck) {
				return DataStreamIn(path, readVersionCheck);
			},
			PYBA("path"), PYBA("read_version_check")
		)
		;
}
#endif