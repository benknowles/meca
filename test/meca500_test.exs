defmodule Meca500Test do
  use ExUnit.Case

  test "answer codes" do
    # standard command (ignores state)
    assert Meca500.answer_codes("BrakesOn", %{}) == [2010]
    assert Meca500.answer_codes("BrakesOn", %{eob: 1, eom: 1}) == [2010]

    # pause motion EOM special case
    assert Meca500.answer_codes("PauseMotion", %{}) == [2042]
    assert Meca500.answer_codes("PauseMotion", %{eom: 1}) == [2042, 3004]

    # appended answer codes for EOB and EOM 
    assert Meca500.answer_codes("SetWRF", %{eob: 1, eom: 1}) == [3004, 3012]
    assert Meca500.answer_codes("SetWRF", %{eob: 1, eom: 0}) == [3012]
    assert Meca500.answer_codes("SetWRF", %{eob: 0, eom: 1}) == [3004]
    assert Meca500.answer_codes("SetWRF", %{eob: 0, eom: 0}) == []

    # non EOM commands with EOM set do not include 3004
    assert Meca500.answer_codes("SetAutoConf", %{eob: 0, eom: 1}) == []
    assert Meca500.answer_codes("SetAutoConf", %{eob: 1, eom: 1}) == [3012]
  end

  test "build commands" do
    assert Meca500.build_command("ActivateRobot", []) == "ActivateRobot"

    assert Meca500.build_command("MoveLin", [170, -50, 150, 0, 90, 0]) ==
             "MoveLin(170,-50,150,0,90,0)"
  end
end
