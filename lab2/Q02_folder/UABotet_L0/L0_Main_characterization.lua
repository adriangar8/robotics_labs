-- L0 Characterization of UABotet
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    September 2023
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

init = function()
  robot = nil -- Object handle
  if sim then robot = sim.getObject("..") end
  C = {}      -- Coordinates (X, Y, angle w.r.t. Z)  
  T = 0       -- Time
  state = {}; state.next = "START"
  B = {}; B.next = 0 -- Begin time 
  X = {}; X.next = 0 -- X position
  Y = {}; Y.next = 0 -- Y position
  Z = {}; Z.next = 0 -- orientation
  L = {}; L.next = 0 -- DC value for left motor
  R = {}; R.next = 0 -- DC value for right motor
  V = {}; V.next = 0 -- Linear speed
  W = {}; W.next = 0 -- Angular speed 
end -- init()
forward = function()
  state.curr = state.next
  B.curr = B.next
  X.curr = X.next
  Y.curr = Y.next
  Z.curr = Z.next
  L.curr = L.next
  R.curr = R.next
  V.curr = V.next
  W.curr = W.next
end -- forward()
read_inputs = function()
  if sim then
    T = sim.getSimulationTime()
    local position = sim.getObjectPosition(robot, sim.handle_world)
    local eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
    C[1] = position[1]; C[2] = position[2]; C[3] = eulerAngles[3]
  else -- console input
    T = T + 0.05
    io.write(string.format("> T = %.4fs or ...", T))
    local newT = tonumber(io.read())
    if newT then T = newT end
    io.write(string.format("> X = "))
    C[1] = tonumber(io.read())
    if C[1] then
      io.write(string.format("> Y = "))
      C[2] = tonumber(io.read())
      if C[2] then
        io.write(string.format("> orientation = "))
        C[3] = tonumber(io.read())
        if not C[3] then C[1] = nil end
      else
        C[1] = nil
      end -- if
    end -- if
  end -- if
end -- read_inputs()
write_outputs = function()
  print(string.format("< L= %i, R= %i, V= %.2fcm/s, W= %.2fdeg/s\n",
      L.curr, R.curr, V.curr, W.curr))
  if sim then
    sim.setInt32Signal("DC_left", L.curr)
    sim.setInt32Signal("DC_right", R.curr)
  end -- if 
end -- write_outputs()
step = function()
  if not sim and C[1]==nil then state.next = "STOP"; state.curr = "STOP" end
  if state.curr=="START" then
    B.next = T; X.next = C[1]; Y.next = C[2]
    L.next = 100; R.next = 100
    state.next = "FWD"
  elseif state.curr=="FWD" then
    local delay = T-B.curr
    if delay>4 then
      local dX = C[1]-X.curr
      local dY = C[2]-Y.curr
      V.next = 100*math.sqrt(dX*dX+dY*dY)/delay
      B.next = T; Z.next = C[3]
      L.next = -100; R.next = 100
      state.next = "ROT"
    end -- if
  elseif state.curr=="ROT" then
    local delay = T-B.curr
    if delay>4 then
      local dA = math.abs(C[3]-Z.curr)
      W.next = 180*dA/delay/math.pi
      L.next = 0; R.next = 0
      state.next = "STOP"
    end -- if
  elseif state.curr== "STOP" then
  else -- Error
      state.next = "STOP"
  end -- if..ifelse
end -- step()

if not sim then -- LOCAL SIMULATION ENGINE
  init()
  forward()
  while state.curr~="STOP" do
    write_outputs()
    read_inputs()
    step()
    forward()
  end -- while
end -- if
