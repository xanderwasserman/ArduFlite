-- HZN.lua - ArduFlite Artificial Horizon for RadioMaster Pocket (EdgeTX)
-- Displays attitude horizon, altitude, and flight mode from CRSF telemetry

--------------------------------------------------------------------------------
-- Configuration
--------------------------------------------------------------------------------
local PITCH_SRC = "Ptch"   -- CRSF attitude pitch sensor
local ROLL_SRC  = "Roll"   -- CRSF attitude roll sensor
local ALT_SRC   = "Alt"    -- CRSF barometric altitude sensor
local FM_SRC    = "FM"     -- CRSF flight mode sensor

-- Invert if horizon moves opposite to aircraft
local INVERT_PITCH = false
local INVERT_ROLL  = true

-- Sensor field IDs (resolved at init)
local pitchId = nil
local rollId = nil
local altId = nil
local fmId = nil

--------------------------------------------------------------------------------
-- Helpers
--------------------------------------------------------------------------------
local function clamp(x, lo, hi)
  return (x < lo) and lo or (x > hi) and hi or x
end

local function round(x)
  return math.floor(x + 0.5)
end

--------------------------------------------------------------------------------
-- Init function (called once when script loads)
--------------------------------------------------------------------------------
local function init()
  -- Resolve sensor field IDs
  local pInfo = getFieldInfo(PITCH_SRC)
  local rInfo = getFieldInfo(ROLL_SRC)
  local aInfo = getFieldInfo(ALT_SRC)
  local fInfo = getFieldInfo(FM_SRC)

  pitchId = pInfo and pInfo.id or nil
  rollId  = rInfo and rInfo.id or nil
  altId   = aInfo and aInfo.id or nil
  fmId    = fInfo and fInfo.id or nil
end

--------------------------------------------------------------------------------
-- Main run function (called every frame by EdgeTX)
--------------------------------------------------------------------------------
local function run(event)
  lcd.clear()

  -- Check if sensors were found
  if pitchId == nil or rollId == nil then
    lcd.drawText(2, 10, "Sensors not found!", 0)
    lcd.drawText(2, 22, "Expected: " .. PITCH_SRC .. ", " .. ROLL_SRC, SMLSIZE)
    lcd.drawText(2, 34, "Check Telemetry->Sensors", SMLSIZE)
    lcd.drawText(2, 46, "for actual names", SMLSIZE)
    return 0
  end

  -- Read telemetry values (EdgeTX already converts CRSF to degrees/meters)
  local pitch = getValue(pitchId) or 0
  local roll  = getValue(rollId) or 0
  local alt   = altId and getValue(altId) or 0
  local fm    = fmId and getValue(fmId) or nil

  if INVERT_PITCH then pitch = -pitch end
  if INVERT_ROLL  then roll  = -roll  end

  -- Clamp for display sanity
  pitch = clamp(pitch, -90, 90)
  roll  = clamp(roll, -180, 180)

  ----------------------------------------------------------------------------
  -- Draw horizon with ground fill
  ----------------------------------------------------------------------------
  local cx = math.floor(LCD_W / 2)
  local cy = math.floor(LCD_H / 2)

  -- Pitch: pixels per degree (adjust for desired sensitivity)
  local pitchScale = 0.8
  local pitchOffset = pitch * pitchScale

  -- Roll: horizon line angle (clamp to avoid tan() explosion)
  local rollClamped = clamp(roll, -85, 85)
  local rollRad = rollClamped * math.pi / 180
  local slope = math.tan(rollRad)

  -- Helper: get horizon Y at given X
  local function horizonY(x)
    return cy + pitchOffset + (x - cx) * slope
  end

  -- Fill ground (below horizon) with black using scanlines
  -- For each row, find where horizon crosses and fill appropriately
  for y = 0, LCD_H - 1 do
    local hLeft = horizonY(0)
    local hRight = horizonY(LCD_W - 1)
    
    -- Determine fill range for this row
    local fillStart = nil
    local fillEnd = nil
    
    if y >= hLeft and y >= hRight then
      -- Entire row below horizon
      fillStart = 0
      fillEnd = LCD_W - 1
    elseif y < hLeft and y < hRight then
      -- Entire row above horizon - no fill
    else
      -- Partial row - find intersection
      local xIntersect = cx + (y - cy - pitchOffset) / slope
      xIntersect = clamp(round(xIntersect), 0, LCD_W - 1)
      
      if hLeft > hRight then
        -- Slope negative: ground on left side
        if y >= hRight then
          fillStart = xIntersect
          fillEnd = LCD_W - 1
        end
      else
        -- Slope positive: ground on right side  
        if y >= hLeft then
          fillStart = 0
          fillEnd = xIntersect
        end
      end
    end
    
    if fillStart and fillEnd and fillStart <= fillEnd then
      lcd.drawLine(fillStart, y, fillEnd, y, SOLID, FORCE)
    end
  end

  -- Draw horizon line (white on top of black ground)
  local y1 = round(horizonY(0))
  local y2 = round(horizonY(LCD_W - 1))
  -- Clamp to screen for stability
  y1 = clamp(y1, -100, LCD_H + 100)
  y2 = clamp(y2, -100, LCD_H + 100)
  lcd.drawLine(0, y1, LCD_W - 1, y2, SOLID, 0)

  -- Draw 10° pitch ladder lines
  for deg = -30, 30, 10 do
    if deg ~= 0 then
      local ladderY = cy + (pitch - deg) * pitchScale
      local ladderHalfW = 8
      local ly1 = round(ladderY - ladderHalfW * slope)
      local ly2 = round(ladderY + ladderHalfW * slope)
      -- Determine color based on position relative to horizon
      local ladderColor = (ladderY > cy + pitchOffset) and 0 or FORCE
      lcd.drawLine(cx - ladderHalfW, ly1, cx + ladderHalfW, ly2, DOTTED, ladderColor)
    end
  end

  -- Aircraft reference (center crosshair) - adaptive color
  -- If center is below horizon (in ground), use 0 to XOR/erase; else FORCE for solid black
  local centerInGround = (pitch < 0)  -- nose down = center is in ground
  local markerColor = centerInGround and 0 or FORCE
  
  lcd.drawLine(cx - 10, cy, cx - 4, cy, SOLID, markerColor)
  lcd.drawLine(cx + 4, cy, cx + 10, cy, SOLID, markerColor)
  lcd.drawLine(cx, cy - 2, cx, cy + 2, SOLID, markerColor)
  -- Add small filled circle/dot at center for visibility
  lcd.drawLine(cx - 1, cy, cx + 1, cy, SOLID, markerColor)

  ----------------------------------------------------------------------------
  -- Info overlay
  ----------------------------------------------------------------------------
  -- Top left: Pitch/Roll numeric (on sky, usually white background)
  lcd.drawText(1, 1, string.format("P%+d R%+d", round(pitch), round(roll)), SMLSIZE)

  -- Top right: Altitude
  if alt then
    local altStr = string.format("%dm", round(alt))
    lcd.drawText(LCD_W - 1, 1, altStr, SMLSIZE + RIGHT)
  end

  -- Bottom center: Flight mode (may be on ground, use inverse if needed)
  if fm and type(fm) == "string" and #fm > 0 then
    local fmW = #fm * 5
    local fmY = LCD_H - 9
    local fmColor = (horizonY(cx) < fmY) and INVERS or 0
    lcd.drawText(cx - math.floor(fmW / 2), fmY, fm, SMLSIZE + fmColor)
  end

  return 0
end

--------------------------------------------------------------------------------
-- EdgeTX script interface
--------------------------------------------------------------------------------
return { init = init, run = run }
