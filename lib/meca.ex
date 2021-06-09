defmodule Meca do
  use GenServer

  @moduledoc """
  Module for communicating and controlling a Mecademic Robot over TCP.

  ## Example

      {:ok, pid} = Meca.start_link(%{host: '127.0.0.1', port: 10000})

      Meca.activate_robot(pid)
      Meca.home(pid)
      Meca.set_blending(pid, 0)
      Meca.set_joint_vel(pid, 100)

      Meca.move_joints(pid, 0, 0, 0, 170, 115, 175)
      Meca.move_joints(pid, 0, 0, 0, -170, -115, -175)
      Meca.move_joints(pid, 0, -70, 70, 0, 0, 0)
      Meca.move_joints(pid, 0, 90, -135, 0, 0, 0)

      Meca.gripper_close(pid)
  """

  @initial_state %{
    socket: nil,
    host: nil,
    port: nil,
    eob: 1,
    eom: 1,
    user_eom: nil,
    error_mode: false,
    queue: false
  }

  @eom_commands [
    "MoveJoints",
    "MoveLin",
    "MoveLinRelTRF",
    "MoveLinRelWRF",
    "MovePose",
    "SetCartAcc",
    "SetJointAcc",
    "SetTRF",
    "SetWRF"
  ]

  @robot_status_keys [
    :activated,
    :homing,
    :simulation,
    :error,
    :paused,
    :eob,
    :eom
  ]

  @gripper_status_keys [
    :gripper_enabled,
    :homing_state,
    :holding_part,
    :limit_reached,
    :error_state,
    :force_overload
  ]

  @float_response_codes [
    2026,
    2027
  ]

  @integer_response_codes [
    2029,
    2007,
    2079
  ]

  @type standard_command_response :: String.t()

  @spec run_command(String.t(), pid()) :: standard_command_response()

  def run_command(cmd, pid) do
    GenServer.call(pid, {:command, cmd})
  end

  def start_link(opts \\ %{}) do
    GenServer.start_link(__MODULE__, Map.merge(@initial_state, opts))
  end

  @doc false
  def init(state) do
    opts = [:binary, packet: :line, line_delimiter: 0, buffer: 1024, active: false]
    {:ok, socket} = :gen_tcp.connect(state[:host], state[:port], opts)
    {:ok, %{state | socket: socket}}
  end

  @doc false
  def handle_call({:command, cmd}, _, %{socket: socket} = state) do
    :ok = :gen_tcp.send(socket, encode(cmd))

    {:ok, resp} = :gen_tcp.recv(socket, 0)

    {:reply, resp, state}
  end

  @spec encode(String.t()) :: String.t()

  def encode(msg) do
    "#{msg}\0"
  end

  @spec decode(String.t()) :: String.t()

  def decode(msg) do
    msg
  end

  @spec activate_robot(pid()) :: standard_command_response()

  @doc """
  Activates the Mecademic Robot.
  """
  def activate_robot(pid), do: "ActivateRobot" |> run_command(pid)

  @spec deactivate_robot(pid()) :: standard_command_response()

  @doc """
  Deactivates the Mecademic Robot.
  """
  def deactivate_robot(pid), do: "DeactivateRobot" |> run_command(pid)

  @spec activate_sim(pid()) :: standard_command_response()

  @doc """
  Activates the Mecademic Robot simulation mode.
  """
  def activate_sim(pid), do: "ActivateSim" |> run_command(pid)

  @spec deactivate_sim(pid()) :: standard_command_response()

  @doc """
  Deactivates the Mecademic Robot simulation mode.
  """
  def deactivate_sim(pid), do: "DeactivateSim" |> run_command(pid)

  @spec switch_to_ethercat(pid()) :: standard_command_response()

  @doc """
  Places the Mecademic Robot in EtherCAT mode.
  """
  def switch_to_ethercat(pid), do: "SwitchToEtherCAT" |> run_command(pid)

  @spec get_conf(pid()) :: standard_command_response()

  def get_conf(pid), do: "GetConf" |> run_command(pid)

  @spec get_joints(pid()) :: standard_command_response()

  def get_joints(pid), do: "GetJoints" |> run_command(pid)

  @spec get_pose(pid()) :: standard_command_response()

  def get_pose(pid), do: "GetPose" |> run_command(pid)

  @spec pause_motion(pid()) :: standard_command_response()

  def pause_motion(pid), do: "PauseMotion" |> run_command(pid)

  @spec resume_motion(pid()) :: standard_command_response()

  def resume_motion(pid), do: "ResumeMotion" |> run_command(pid)

  @spec clear_motion(pid()) :: standard_command_response()

  def clear_motion(pid), do: "ClearMotion" |> run_command(pid)

  @spec brakes_on(pid()) :: standard_command_response()

  def brakes_on(pid), do: "BrakesOn" |> run_command(pid)

  @spec brakes_off(pid()) :: standard_command_response()

  def brakes_off(pid), do: "BrakesOff" |> run_command(pid)

  @spec home(pid()) :: standard_command_response()

  @doc """
  Homes the Mecademic Robot.
  """
  def home(pid), do: "Home" |> run_command(pid)

  @spec gripper_open(pid()) :: standard_command_response()

  @doc """
  Opens the gripper of the end-effector.
  """
  def gripper_open(pid), do: "GripperOpen" |> run_command(pid)

  @spec gripper_close(pid()) :: standard_command_response()

  @doc """
  Closes the gripper of the end-effector.
  """
  def gripper_close(pid), do: "GripperClose" |> run_command(pid)

  @spec get_status_robot(pid()) :: %{atom() => integer()}

  def get_status_robot(pid) do
    "GetStatusRobot"
    |> run_command(pid)
    |> (&Enum.zip(@robot_status_keys, &1)).()
    |> Enum.into(%{})
  end

  @spec get_status_gripper(pid()) :: %{atom() => integer()}

  def get_status_gripper(pid) do
    "GetStatusGripper"
    |> run_command(pid)
    |> (&Enum.zip(@gripper_status_keys, &1)).()
    |> Enum.into(%{})
  end

  @spec in_error_mode?(pid()) :: boolean()

  @doc """
  Status method that checks whether the Mecademic Robot is in error mode.
  """
  def in_error_mode?(pid) do
    :sys.get_state(pid) |> Map.get(:error_mode)
  end

  @spec set_eob(pid(), integer()) :: standard_command_response()

  @doc """
  Sets End of Block answer active or inactive in the Mecademic Robot. Parameter `e`
  enables (1) EOB or disables (0) EOB.
  """
  def set_eob(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eob: e} end)
    build_command("SetEOB", [e]) |> run_command(pid)
  end

  @spec set_eom(pid(), integer()) :: standard_command_response()

  @doc """
  Sets End of Movement answer active or inactive in the Mecademic Robot. Parameter `e`
  enables (1) EOM or disables (0) EOM.
  """
  def set_eom(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eom: e} end)
    build_command("SetEOM", [e]) |> run_command(pid)
  end

  @spec set_queue(pid(), integer()) :: boolean()

  @doc """
  Enables the queueing of move commands for blending. Parameter `e` enables (1) queueing
  or disables (0) queueing.
  """
  def set_queue(pid, e) do
    if e == 1 do
      eom = :sys.get_state(pid) |> Map.get(:eom)

      :sys.replace_state(pid, fn state ->
        %{state | queue: true, user_eom: eom}
      end)

      set_eom(pid, 0)
    else
      user_eom = :sys.get_state(pid) |> Map.get(:user_eom)

      :sys.replace_state(pid, fn state ->
        %{state | queue: false}
      end)

      set_eom(pid, user_eom)
    end

    :sys.get_state(pid) |> Map.get(:queue)
  end

  @spec delay(pid(), integer()) :: standard_command_response()

  @doc """
  Gives the Mecademic Robot a wait time before performing another action.
  """
  def delay(pid, t) do
    build_command("Delay", [t]) |> run_command(pid)
  end

  @spec move_joints(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  @doc """
  Moves the joints of the Mecademic Robot to the desired angles. Each theta argument corresponds
  to a joint number.
  """
  def move_joints(pid, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6) do
    build_command("MoveJoints", [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
    |> run_command(pid)
  end

  @spec move_lin(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  def move_lin(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLin", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec move_lin_rel_trf(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  def move_lin_rel_trf(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLinRelTRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec move_lin_rel_wrf(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  def move_lin_rel_wrf(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLinRelWRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec move_pose(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  def move_pose(pid, x, y, z, alpha, beta, gamma) do
    build_command("MovePose", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec set_blending(pid(), float()) :: standard_command_response()

  def set_blending(pid, p) do
    build_command("SetBlending", [p]) |> run_command(pid)
  end

  @spec set_auto_conf(pid(), integer()) :: standard_command_response()

  def set_auto_conf(pid, e) do
    build_command("SetAutoConf", [e]) |> run_command(pid)
  end

  @spec set_cart_acc(pid(), float()) :: standard_command_response()

  def set_cart_acc(pid, p) do
    build_command("SetCartAcc", [p]) |> run_command(pid)
  end

  @spec set_cart_ang_vel(pid(), float()) :: standard_command_response()

  def set_cart_ang_vel(pid, w) do
    build_command("SetCartAngVel", [w]) |> run_command(pid)
  end

  @spec set_cart_lin_vel(pid(), float()) :: standard_command_response()

  def set_cart_lin_vel(pid, v) do
    build_command("SetCartLinVel", [v]) |> run_command(pid)
  end

  @spec set_conf(pid(), integer(), integer(), integer()) :: standard_command_response()

  @doc """
  Sets the desired Mecademic Robot inverse kinematic configuration to be observed in
  the MovePose command. Parameters `c1`, `c3`, and `c5` should be either `1` or `-1`.
  """
  def set_conf(pid, c1, c3, c5) do
    build_command("SetConf", [c1, c3, c5]) |> run_command(pid)
  end

  @spec set_gripper_force(pid(), float()) :: standard_command_response()

  @doc """
  Sets the Gripper's grip force. Parameter `p` should be between `1` and `100`.
  """
  def set_gripper_force(pid, p) do
    build_command("SetGripperForce", [p]) |> run_command(pid)
  end

  @spec set_gripper_vel(pid(), float()) :: standard_command_response()

  @doc """
  Sets the Gripper fingers' velocity with respect to the gripper. Parameter `p` should be
  between `1` and `100`.
  """
  def set_gripper_vel(pid, p) do
    build_command("SetGripperVel", [p]) |> run_command(pid)
  end

  @spec set_joint_acc(pid(), float()) :: standard_command_response()

  @doc """
  Sets the acceleration of the joints. Parameter `p` should be between `1` and `100`.
  """
  def set_joint_acc(pid, p) do
    build_command("SetJointAcc", [p]) |> run_command(pid)
  end

  @spec set_joint_vel(pid(), float()) :: standard_command_response()

  @doc """
  Sets the angular velocities of the Mecademic Robot's joints. `velocity` should be
  between `1` and `100`.
  """
  def set_joint_vel(pid, velocity) do
    build_command("SetJointVel", [velocity]) |> run_command(pid)
  end

  @spec set_trf(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  @doc """
  Sets the Mecademic Robot TRF at (x, y, z) and heading (alpha, beta, gamma) with respect
  to the FRF.
  """
  def set_trf(pid, x, y, z, alpha, beta, gamma) do
    build_command("SetTRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec set_wrf(pid(), float(), float(), float(), float(), float(), float()) ::
          standard_command_response()

  @doc """
  Sets the Mecademic Robot WRF at (x, y, z) and heading (alpha, beta, gamma) with respect
  to the BRF.
  """
  def set_wrf(pid, x, y, z, alpha, beta, gamma) do
    build_command("SetWRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec build_command(String.t(), list(integer() | float() | String.t())) ::
          standard_command_response()

  @doc """
  Builds the command string to send to the Mecademic Robot from the function name and
  arguments the command needs.
  """
  def build_command(command, args) do
    case Enum.count(args) do
      0 ->
        command

      _ ->
        args_list = Enum.join(args, ",")
        "#{command}(#{args_list})"
    end
  end

  @spec parse_response(String.t()) :: {integer(), String.t()}

  def parse_response(resp) do
    trimmed = resp |> String.trim("\0")
    {parse_response_code(trimmed), parse_response_body(trimmed)}
  end

  @spec parse_response_code(String.t()) :: integer()

  defp parse_response_code(resp) do
    resp |> String.slice(1..4) |> String.to_integer()
  end

  @spec parse_response_body(String.t()) :: String.t()

  defp parse_response_body(resp) do
    resp |> String.slice(7..-2)
  end

  @spec decode_response_body(integer(), String.t()) ::
          list(float()) | list(integer()) | String.t()

  def decode_response_body(code, body) do
    cond do
      Enum.member?(@float_response_codes, code) ->
        body
        |> String.split(",")
        |> Enum.map(&String.to_float/1)

      Enum.member?(@integer_response_codes, code) ->
        body
        |> String.split(",")
        |> Enum.map(&String.to_integer/1)

      true ->
        body
    end
  end

  @spec answer_codes(String.t(), %{optional(:eob) => integer(), optional(:eom) => integer()}) ::
          list(integer())
  @doc """
  Returns the list of possible answer codes for the given command and EOM/EOB enablement status.
  """
  def answer_codes(command, state)
  def answer_codes("ActivateRobot", _), do: [2000, 2001]
  def answer_codes("ActivateSim", _), do: [2045]
  def answer_codes("ClearMotion", _), do: [2044]
  def answer_codes("DeactivateRobot", _), do: [2004]
  def answer_codes("BrakesOn", _), do: [2010]
  def answer_codes("BrakesOff", _), do: [2008]
  def answer_codes("GetConf", _), do: [2029]
  def answer_codes("GetJoints", _), do: [2026]
  def answer_codes("GetStatusRobot", _), do: [2007]
  def answer_codes("GetStatusGripper", _), do: [2079]
  def answer_codes("GetPose", _), do: [2027]
  def answer_codes("Home", _), do: [2002, 2003]
  def answer_codes("PauseMotion", %{eom: 1}), do: [2042, 3004]
  def answer_codes("PauseMotion", _), do: [2042]
  def answer_codes("ResetError", _), do: [2005, 2006]
  def answer_codes("ResumeMotion", _), do: [2043]
  def answer_codes("SetEOB", _), do: [2054, 2055]
  def answer_codes("SetEOM", _), do: [2052, 2053]

  def answer_codes(command, %{eob: eob, eom: eom}) do
    []
    |> append_eob_answer_code(eob)
    |> append_eom_answer_code(command, eom)
  end

  def answer_codes(_, _), do: []

  @spec append_eob_answer_code(list(integer()), integer()) :: list(integer())

  defp append_eob_answer_code(list, 1), do: [3012 | list]
  defp append_eob_answer_code(list, _), do: list

  @spec append_eom_answer_code(list(integer()), String.t(), integer()) :: list(integer())

  defp append_eom_answer_code(list, command, 1) do
    if Enum.member?(@eom_commands, command) do
      [3004 | list]
    else
      list
    end
  end

  defp append_eom_answer_code(list, _, _), do: list
end
