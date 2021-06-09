defmodule Meca do
  use GenServer

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

  def xyz do
    {:ok, pid} = Meca.start_link(%{host: '127.0.0.1', port: 10000})
    Meca.activate_robot(pid)
    Meca.set_eob(pid, 0)
    Meca.activate_robot(pid)
  end

  def run_command(cmd, pid) do
    GenServer.call(pid, {:command, cmd})
  end

  def start_link(opts \\ %{}) do
    GenServer.start_link(__MODULE__, Map.merge(@initial_state, opts))
  end

  def init(state) do
    opts = [:binary, packet: :line, line_delimiter: 0, buffer: 1024, active: false]
    {:ok, socket} = :gen_tcp.connect(state[:host], state[:port], opts)
    {:ok, %{state | socket: socket}}
  end

  def handle_call({:command, cmd}, _, %{socket: socket} = state) do
    :ok = :gen_tcp.send(socket, encode(cmd))

    {:ok, resp} = :gen_tcp.recv(socket, 0)

    {:reply, resp, state}
  end

  def encode(msg) do
    "#{msg}\0"
  end

  def decode(msg) do
    msg
  end

  def activate_robot(pid), do: "ActivateRobot" |> run_command(pid)

  def deactivate_robot(pid), do: "DeactivateRobot" |> run_command(pid)

  def activate_sim(pid), do: "ActivateSim" |> run_command(pid)

  def deactivate_sim(pid), do: "DeactivateSim" |> run_command(pid)

  def switch_to_ethercat(pid), do: "SwitchToEtherCAT" |> run_command(pid)

  def get_conf(pid), do: "GetConf" |> run_command(pid)

  def get_joints(pid), do: "GetJoints" |> run_command(pid)

  def get_pose(pid), do: "GetPose" |> run_command(pid)

  def pause_motion(pid), do: "PauseMotion" |> run_command(pid)

  def resume_motion(pid), do: "ResumeMotion" |> run_command(pid)

  def clear_motion(pid), do: "ClearMotion" |> run_command(pid)

  def brakes_on(pid), do: "BrakesOn" |> run_command(pid)

  def brakes_off(pid), do: "BrakesOff" |> run_command(pid)

  def home(pid), do: "Home" |> run_command(pid)

  def gripper_open(pid), do: "GripperOpen" |> run_command(pid)

  def gripper_close(pid), do: "GripperClose" |> run_command(pid)

  def get_status_robot(pid) do
    "GetStatusRobot"
    |> run_command(pid)
    |> (&Enum.zip(@robot_status_keys, &1)).()
    |> Enum.into(%{})
  end

  def get_status_gripper(pid) do
    "GetStatusGripper"
    |> run_command(pid)
    |> (&Enum.zip(@gripper_status_keys, &1)).()
    |> Enum.into(%{})
  end

  def in_error_mode?(pid) do
    :sys.get_state(pid) |> Map.get(:error_mode)
  end

  def set_eob(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eob: e} end)
    build_command("SetEOB", [e]) |> run_command(pid)
  end

  def set_eom(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eom: e} end)
    build_command("SetEOM", [e]) |> run_command(pid)
  end

  def set_queue(pid, e) do
    cond do
      e == 1 ->
        eom = :sys.get_state(pid) |> Map.get(:eom)

        :sys.replace_state(pid, fn state ->
          %{state | queue: true, user_eom: eom}
        end)

        set_eom(pid, 0)

      true ->
        user_eom = :sys.get_state(pid) |> Map.get(:user_eom)

        :sys.replace_state(pid, fn state ->
          %{state | queue: false}
        end)

        set_eom(pid, user_eom)
    end

    :sys.get_state(pid) |> Map.get(:queue)
  end

  def delay(pid, t) do
    build_command("Delay", [t]) |> run_command(pid)
  end

  def move_joints(pid, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6) do
    build_command("MoveJoints", [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
    |> run_command(pid)
  end

  def move_lin(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLin", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  def move_lin_rel_trf(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLinRelTRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  def move_lin_rel_wrf(pid, x, y, z, alpha, beta, gamma) do
    build_command("MoveLinRelWRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  def move_pose(pid, x, y, z, alpha, beta, gamma) do
    build_command("MovePose", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  def set_blending(pid, p) do
    build_command("SetBlending", [p]) |> run_command(pid)
  end

  def set_auto_conf(pid, e) do
    build_command("SetAutoConf", [e]) |> run_command(pid)
  end

  def set_cart_acc(pid, p) do
    build_command("SetCartAcc", [p]) |> run_command(pid)
  end

  def set_cart_ang_vel(pid, w) do
    build_command("SetCartAngVel", [w]) |> run_command(pid)
  end

  def set_cart_lin_vel(pid, v) do
    build_command("SetCartLinVel", [v]) |> run_command(pid)
  end

  def set_conf(pid, c1, c3, c5) do
    build_command("SetConf", [c1, c3, c5]) |> run_command(pid)
  end

  def set_gripper_force(pid, p) do
    build_command("SetGripperForce", [p]) |> run_command(pid)
  end

  def set_gripper_vel(pid, p) do
    build_command("SetGripperVel", [p]) |> run_command(pid)
  end

  def set_joint_acc(pid, p) do
    build_command("SetJointAcc", [p]) |> run_command(pid)
  end

  @spec set_joint_vel(pid(), float()) :: String.t()

  def set_joint_vel(pid, velocity) do
    build_command("SetJointVel", [velocity]) |> run_command(pid)
  end

  @spec set_trf(pid(), float(), float(), float(), float(), float(), float()) :: String.t()

  def set_trf(pid, x, y, z, alpha, beta, gamma) do
    build_command("SetTRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec set_wrf(pid(), float(), float(), float(), float(), float(), float()) :: String.t()

  def set_wrf(pid, x, y, z, alpha, beta, gamma) do
    build_command("SetWRF", [x, y, z, alpha, beta, gamma]) |> run_command(pid)
  end

  @spec build_command(String.t(), list(integer() | float() | String.t())) :: String.t()

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
    cond do
      Enum.member?(@eom_commands, command) ->
        [3004 | list]

      true ->
        list
    end
  end

  defp append_eom_answer_code(list, _, _), do: list
end
