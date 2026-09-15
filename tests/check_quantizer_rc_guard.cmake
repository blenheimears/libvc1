if(NOT DEFINED SRC)
  message(FATAL_ERROR "SRC not supplied")
endif()
file(READ "${SRC}/src/encoder_core.cpp" C)
foreach(NEEDED
    "struct QuantChoice"
    "scale_to_quant"
    "d.halfqp=quant.halfqp"
    "estimate_picture_quantizer_rate"
    "auto qrate=qestimator.estimate_picture_quantizer_rate(f)"
    "if (aq_contextual_integer_picture)"
    "qrate.selected=libvc1::QuantizerType::Uniform"
    "if (aq_contextual_integer_picture) rc_cfg.allow_halfqp=false"
    "const double coding_law_scale=std::clamp(qrate.rate_scale*transform_scale,0.85,1.15)"
    "qcfg.halfqp=resolved_halfqp"
    "selector.choose_picture_quantizer_type(f)")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "ABR quantizer parity marker missing: ${NEEDED}")
  endif()
endforeach()
foreach(FORBIDDEN
    "automatic ABR half-step selection remains pending"
    "preserve the historical uniform law in ABR"
    "tcfg.halfqp=false")
  string(FIND "${C}" "${FORBIDDEN}" POS)
  if(NOT POS EQUAL -1)
    message(FATAL_ERROR "obsolete ABR quantizer compatibility guard remains: ${FORBIDDEN}")
  endif()
endforeach()
message(STATUS "ABR HALFQP/nonuniform quantizer parity structural checks ok")
