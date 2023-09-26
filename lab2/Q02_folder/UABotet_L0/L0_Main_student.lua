-- L0 Open-loop control of UABotet
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    September 2023
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

-- L0 Main
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    September 2023
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

-- L0 UI interface
L0_UI = {}
L0_UI.xml = [[
  <ui title="L0 User Interface" resizable="true"
      style="background-color:lightBlue"
  >
    <group layout="hbox" flat="true">
    <group layout="vbox" flat="true">
    <group layout="hbox" flat="true">
    <label text="Angle" />
    <edit	id="110" value="0"
      on-editing-finished="L0_UI.updateAtext"
      style="background-color:white"
    />
    <label text="deg" />
    </group>
    <hslider id="115" minimum="-90" maximum="90" value="0" 
      tick-interval="10" tick-position="below" on-change="L0_UI.updateAslider" />
    <group layout="hbox" flat="true">
    <label text="Distance" />
    <edit	id="120" value="0"
      on-editing-finished="L0_UI.updateDtext"
      style="background-color:white"
    />
    <label text="cm" />
    </group>        
    <hslider id="125" minimum="0" maximum="255" value="0" 
      tick-interval="10" tick-position="below" on-change="L0_UI.updateDslider" />
    <group layout="hbox" flat="true">
    <button id="1" text="GO" style="background-color:lightGreen"
      on-click="L0_UI.GO" />
    <button id="4" text="HALT" style="background-color:red"
      on-click="L0_UI.HALT" />
    </group>
    <group layout="hbox" flat="true">
    <button id="2" text="LIDAR" style="background-color:lightGreen"
      on-click="L0_UI.LIDAR" />
    <button id="3" text="RESUME" style="background-color:lightGreen"
      on-click="L0_UI.RESUME" />
    </group>
    </group>

    <group layout="vbox" flat="true">
    <label text="From L0: "/>
    <text-browser id="40" text="..."
       html="false" read-only="true" style="align:left"
    />
    <!--tree id="40" show-header="false" style="align:left">
      <row> <item> ... </item> </row>
    </tree-->
    </group>

    </group>
  </ui>
]]
if sim then
  L0_UI.handle = simUI.create(L0_UI.xml)
  simUI.setPosition(L0_UI.handle, 50, 450)
end -- if
L0_UI.angle = 0
L0_UI.radius = 0
L0_UI.C = nil
L0_UI.L0msg = "..."
L0_UI.updateAtext = function(uiHandle, id, newValue)
  local angle = tonumber(newValue)
  if angle then
    if angle < -90 then angle = -90 end
    if angle > 90 then angle = 90 end
    L0_UI.angle = angle
    simUI.setSliderValue(uiHandle, 115, angle)
  else
    angle = L0_UI.angle
  end
  simUI.setEditValue(uiHandle, 110, string.format("%d", angle))
end
L0_UI.updateAslider = function(uiHandle, id, newValue)
  L0_UI.angle = newValue
  simUI.setEditValue(uiHandle, 110, string.format("%d", newValue))
end
L0_UI.updateDtext = function(uiHandle, id, newValue)
  local radius = tonumber(newValue)
  if radius then
    if radius < 0 then radius = 0 end
    if radius > 255 then radius = 255 end
    L0_UI.radius = radius
    simUI.setSliderValue(uiHandle, 125, radius)
  else
    radius = L0_UI.radius
  end
  simUI.setEditValue(uiHandle, 120, string.format("%d", radius))
end
L0_UI.updateDslider = function(uiHandle, id, newValue)
  L0_UI.radius = newValue
  simUI.setEditValue(uiHandle, 120, string.format("%d", newValue))
end
L0_UI.GO = function(uiHandle, id)
  L0_UI.C = string.format("1 %d %d", L0_UI.angle, L0_UI.radius)
end
L0_UI.HALT = function(uiHandle, id) L0_UI.C = "4" end
L0_UI.LIDAR = function(uiHandle, id) L0_UI.C = "2" end
L0_UI.RESUME = function(uiHandle, id) L0_UI.C = "3" end
L0_UI.setA = function(self, angle)
  if angle then
    if angle < -90 then angle = -90 end
    if angle > 90 then angle = 90 end
    self.angle = angle
    simUI.setLabelText(self.handle, 110, string.format("%d", angle))
    simUI.setSliderValue(self.handle, 115, angle)
  end
end
L0_UI.setD = function(self, radius)
  if angle then
    if radius < 0 then radius = 0 end
    if radius > 255 then radius = 255 end
    self.radius = radius
    simUI.setLabelText(self.handle, 120, string.format("%d", radius))
    simUI.setSliderValue(self.handle, 125, radius)
  end
end
L0_UI.getC = function(self)
  local C = self.C
  self.C = nil
  return C
end
L0_UI.appendM = function(self, M)
  self.L0msg = M..'\n'..self.L0msg
  simUI.setText(self.handle, 40, self.L0msg)
end

-- L0 CONTROLLER

init = function()
  V =  1.23 -- cm/s  -- REPLACE BY VALUES FROM CoppeliaSim
  W = 45.67 -- deg/s -- SIMULATION !!!!
  I = { 0, 0, 0 } -- Instruction table
  T = 0           -- Time
  state = {}; state.next = "LISTEN"
  B = {}; B.next = 0   -- Begin time 
  A = {}; A.next = nil -- Absolute angle rotation
  S = {}; S.next = nil -- Space/distance to move ahead
  L = {}; L.next = 0   -- DC value for left motor
  R = {}; R.next = 0   -- DC value for right motor
  M = {}; M.next = nil -- Message to user/upper-level controllers
end -- init()

forward = function() -- TO COMPLETE !!!!
  
end -- forward()

read_inputs = function()
  local C = L0_UI:getC()
  if sim then
    T = sim.getSimulationTime()
  else -- console input
    T = T + 0.05
    io.write(string.format("> T = %.2fs or ...", T))
    local newT = tonumber(io.read())
    if newT and newT>T then T = newT end
    io.write(string.format("> I = "))
    C = io.read()
  end -- if
  if C and #C>0 then
      local words = {}
      for w in string.gmatch(C, "-?%d+") do table.insert(words, w) end
      I[1] = tonumber(words[1])
      if I[1] then
        if I[1]==1 then -- GO
          if #words>1 then I[2] = tonumber(words[2]) end
          if #words>2 then I[3] = tonumber(words[3]) end
        else
          I[2] = 0; I[3] = 0
        end -- if
      end -- if
  else
      I[1] = 0 -- no command received
  end -- if
end -- read_inputs()

write_outputs = function()
  if sim then
    --print(string.format("< L= %i, R= %i, M= ", L.curr, R.curr))
    --if M.curr then println(M.curr) else println("nil") end
    if M.curr then L0_UI:appendM(M.curr) end
    if L and L.curr then
      sim.setInt32Signal("DC_left", L.curr)
      sim.setInt32Signal("DC_right", R.curr)
    end -- if
  else
    io.write(string.format("< L= %i, R= %i, M= ", L.curr, R.curr))
    if M.curr then io.write(M.curr.."\n") else io.write("nil\n")  end
  end -- if 
end -- write_outputs()

step = function()
  if not sim and I[1]==nil then state.next = "STOP"; state.curr = "STOP" end
  if state.curr=="LISTEN" then -- TO COMPLETE !!!!
  
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
