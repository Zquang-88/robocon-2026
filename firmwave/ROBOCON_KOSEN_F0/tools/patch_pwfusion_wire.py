from pathlib import Path

Import("env")


libdeps_root = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")
matches = list(libdeps_root.glob("PWFusion_VL53L3C*/src/PWFusion_VL53L3C.cpp"))

if matches:
    source_path = matches[0]
    source = source_path.read_text(encoding="utf-8")
    patched = source.replace(
        "Wire.requestFrom(BASE_ADDR>>1, count)",
        "_i2cPort->requestFrom(BASE_ADDR>>1, count)",
    ).replace("*pData++ = Wire.read();", "*pData++ = _i2cPort->read();")

    write_start = patched.find("bool VL53L3C::write(")
    if write_start >= 0:
        replacement = r'''bool VL53L3C::write(uint16_t index, uint8_t *pData, uint32_t count)
{
  constexpr uint8_t kChunk = 28;
  uint32_t offset = 0;
  while (offset < count) {
    const uint8_t chunk = (uint8_t)min((uint32_t)kChunk, count - offset);
    const uint16_t reg = index + offset;
    _i2cPort->beginTransmission(BASE_ADDR >> 1);
    if (_i2cPort->write(reg >> 8) != 1 ||
        _i2cPort->write(reg & 0xFF) != 1 ||
        _i2cPort->write(pData + offset, chunk) != chunk ||
        _i2cPort->endTransmission(true) != 0) return false;
    offset += chunk;
  }
  return true;
}

bool VL53L3C::read(uint16_t index, uint8_t *pData, uint32_t count)
{
  constexpr uint8_t kChunk = 28;
  uint32_t offset = 0;
  while (offset < count) {
    const uint8_t chunk = (uint8_t)min((uint32_t)kChunk, count - offset);
    const uint16_t reg = index + offset;
    _i2cPort->beginTransmission(BASE_ADDR >> 1);
    if (_i2cPort->write(reg >> 8) != 1 ||
        _i2cPort->write(reg & 0xFF) != 1 ||
        _i2cPort->endTransmission(false) != 0) return false;
    const uint8_t received = _i2cPort->requestFrom(
        (uint8_t)(BASE_ADDR >> 1), chunk);
    if (received != chunk) return false;
    for (uint8_t i = 0; i < chunk; ++i)
      pData[offset + i] = (uint8_t)_i2cPort->read();
    offset += chunk;
  }
  return true;
}
'''
        patched = patched[:write_start] + replacement

    if patched != source:
        source_path.write_text(patched, encoding="utf-8")
        print("Patched PWFusion_VL53L3C to use the selected TwoWire bus")
else:
    print("PWFusion_VL53L3C source not found yet; run the environment again")

platform_matches = list(libdeps_root.glob("PWFusion_VL53L3C*/src/vl53lx_platform.cpp"))
if platform_matches:
    platform_path = platform_matches[0]
    platform = platform_path.read_text(encoding="utf-8")
    platform_patched = platform.replace(
        "((VL53L3C*)(pdev->pParent))->write(index, pdata, count);\n\n\treturn VL53LX_ERROR_NONE;",
        "return ((VL53L3C*)(pdev->pParent))->write(index, pdata, count)\n"
        "    ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;",
    ).replace(
        "((VL53L3C*)(pdev->pParent))->read(index, pdata, count);\n  \n\treturn VL53LX_ERROR_NONE;",
        "return ((VL53L3C*)(pdev->pParent))->read(index, pdata, count)\n"
        "    ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;",
    )
    if platform_patched != platform:
        platform_path.write_text(platform_patched, encoding="utf-8")
        print("Patched PWFusion VL53L3CX platform error propagation")
