if(NOT DEFINED SOURCE)
  message(FATAL_ERROR "SOURCE not supplied")
endif()
file(READ "${SOURCE}" TEXT)
foreach(NEEDED
    "ref.SetReferencedTimecode("
    "cluster.Render(out, unused_cues, false)"
    "cluster.ReleaseFrames()")
  string(FIND "${TEXT}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "missing Matroska safety marker: ${NEEDED}")
  endif()
endforeach()
string(FIND "${TEXT}" "*static_cast<EbmlSInteger*>(&ref)" BAD)
if(NOT BAD EQUAL -1)
  message(FATAL_ERROR "unsafe KaxReferenceBlock base-class assignment returned")
endif()
message(STATUS "Matroska ReferenceBlock API usage check ok")
