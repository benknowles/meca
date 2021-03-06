defmodule Meca do
  use GenServer

  @moduledoc """
  Module for communicating and controlling a Mecademic Robot over TCP.

  ## Example

      {:ok, pid} = Meca.start(%{host: '127.0.0.1', port: 10000})

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
    queueing: false
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

  @reset_error_responses [
    "The error was reset",
    "There was no error to reset"
  ]

  @socket_options [
    :binary,
    packet: :line,
    line_delimiter: 0,
    buffer: 1024,
    active: false
  ]

  @typedoc """
  A response from executing a command.
  """
  @type command_response :: :error_mode | :queueing | String.t() | list(integer()) | list(float())

  @spec run_command(pid(), String.t(), list(integer() | float()) | nil) :: command_response()

  @doc """
  Sends a command to the Mecademic Robot and receives a decoded response.
  """
  def run_command(pid, command, args) do
    GenServer.call(pid, {:command, command, args})
  end

  @spec run_command(pid(), String.t()) :: command_response()

  @doc """
  Sends a command to the Mecademic Robot and receives a decoded response.
  """
  def run_command(pid, command), do: run_command(pid, command, [])

  @spec run_script(pid(), String.t()) :: :ok

  @doc """
  Takes a string of one command on each line, and sends the commands sequentially to
  the Mecademic Robot.

  ## Example

      {:ok, pid} = Meca.start(%{host: '127.0.0.1', port: 10000})

      script = \"\"\"
      SetBlending(0)
      SetJointVel(100)
      MoveJoints(0,0,0,170,115,175)
      MoveJoints(0,0,0,-170,-115,-175)
      MoveJoints(0,0,0,170,115,175)
      \"\"\"

      Meca.run_script(pid, script)
  """
  def run_script(pid, script) do
    script
    |> String.split("\n")
    |> Enum.map(&String.trim/1)
    |> Enum.each(fn command ->
      pid |> run_command(command, nil)
    end)
  end

  @doc """
  Creates a new connection to a Mecademic Robot with the provided connection options.
  Expects a map with a `host` and a `port`.
  """
  def start_link(opts \\ %{}) do
    GenServer.start_link(__MODULE__, Map.merge(@initial_state, opts))
  end

  @doc """
  Creates a new connection to a Mecademic Robot with the provided connection options.
  Expects a map with a `host` and a `port`.
  """
  def start(opts \\ %{}) do
    GenServer.start(__MODULE__, Map.merge(@initial_state, opts))
  end

  @doc false
  def init(state) do
    with {:ok, socket} <- :gen_tcp.connect(state[:host], state[:port], @socket_options, 100),
         {:ok, resp} <- :gen_tcp.recv(socket, 0, 10_000),
         {code, _body} <- parse_response(resp) do
      case code do
        3000 ->
          # connection confirmation
          {:ok, %{state | socket: socket}}

        3001 ->
          # another session is connected to the robot
          {:stop, :existing_connection}

        _ ->
          # unexpected response
          {:stop, {:unexpected_code, code}}
      end
    else
      {:error, err} ->
        {:stop, err}

      err ->
        {:stop, err}
    end
  end

  @doc false
  def handle_call(
        {:command, command, args},
        _,
        %{socket: socket, error_mode: error_mode, queueing: queueing} = state
      ) do
    if error_mode do
      {:reply, :error_mode, state}
    else
      :ok = :gen_tcp.send(socket, build_command(command, args) |> encode())

      if queueing do
        # skip receiving responses if queueing enabled
        {:reply, :queueing, state}
      else
        {:ok, resp} = :gen_tcp.recv(socket, 0)

        {code, body} = parse_response(resp)

        decoded_body = decode_response_body(code, body)

        {:reply, decoded_body, %{state | error_mode: error_code?(code)}}
      end
    end
  end

  @spec encode(String.t()) :: String.t()

  @doc """
  Encodes a message before sending to the Mecademic Robot.
  """
  def encode(msg) do
    "#{msg}\0"
  end

  @spec decode(String.t()) :: String.t()

  @doc """
  Decodes response from the Mecademic Robot into useful information that can be manipulated.
  """
  def decode(msg) do
    msg
  end

  @spec activate_robot(pid()) :: command_response()

  @doc """
  Activates the Mecademic Robot.
  """
  def activate_robot(pid), do: pid |> run_command("ActivateRobot")

  @spec deactivate_robot(pid()) :: command_response()

  @doc """
  Deactivates the Mecademic Robot.
  """
  def deactivate_robot(pid), do: pid |> run_command("DeactivateRobot")

  @spec activate_sim(pid()) :: command_response()

  @doc """
  Activates the Mecademic Robot simulation mode.
  """
  def activate_sim(pid), do: pid |> run_command("ActivateSim")

  @spec deactivate_sim(pid()) :: command_response()

  @doc """
  Deactivates the Mecademic Robot simulation mode.
  """
  def deactivate_sim(pid), do: pid |> run_command("DeactivateSim")

  @spec switch_to_ethercat(pid()) :: command_response()

  @doc """
  Places the Mecademic Robot in EtherCAT mode.
  """
  def switch_to_ethercat(pid), do: pid |> run_command("SwitchToEtherCAT")

  @spec get_conf(pid()) :: command_response()

  @doc """
  Retrieves the current inverse kinematic configuration.
  """
  def get_conf(pid), do: pid |> run_command("GetConf")

  @spec get_joints(pid()) :: command_response()

  @doc """
  Retrieves the Mecademic Robot joint angles in degrees.
  """
  def get_joints(pid), do: pid |> run_command("GetJoints")

  @spec get_pose(pid()) :: command_response()

  @doc """
  Retrieves the current pose of the Mecademic Robot TRF with respect to the WRF.
  """
  def get_pose(pid), do: pid |> run_command("GetPose")

  @spec pause_motion(pid()) :: command_response()

  @doc """
  Stops the robot movement and holds until ResumeMotion.
  """
  def pause_motion(pid), do: pid |> run_command("PauseMotion")

  @spec resume_motion(pid()) :: command_response()

  @doc """
  Resumes the robot movement after being Paused from PauseMotion or ClearMotion.
  """
  def resume_motion(pid), do: pid |> run_command("ResumeMotion")

  @spec clear_motion(pid()) :: command_response()

  @doc """
  Stops the robot movement and deletes the rest of the robot's trajectory.
  Holds until a ResumeMotion.
  """
  def clear_motion(pid), do: pid |> run_command("ClearMotion")

  @spec brakes_on(pid()) :: command_response()

  @doc """
  Enables the brakes of joints 1, 2 and 3, if and only if the robot is powered but deactivated.
  """
  def brakes_on(pid), do: pid |> run_command("BrakesOn")

  @spec brakes_off(pid()) :: command_response()

  @doc """
  Disables the brakes of joints 1, 2 and 3, if and only if the robot is powered but deactivated.
  """
  def brakes_off(pid), do: pid |> run_command("BrakesOff")

  @spec home(pid()) :: command_response()

  @doc """
  Homes the Mecademic Robot.
  """
  def home(pid), do: pid |> run_command("Home")

  @spec gripper_open(pid()) :: command_response()

  @doc """
  Opens the gripper of the end-effector.
  """
  def gripper_open(pid), do: pid |> run_command("GripperOpen")

  @spec gripper_close(pid()) :: command_response()

  @doc """
  Closes the gripper of the end-effector.
  """
  def gripper_close(pid), do: pid |> run_command("GripperClose")

  @spec get_status_robot(pid()) :: %{
          :activated => integer(),
          :homing => integer(),
          :simulation => integer(),
          :error => integer(),
          :paused => integer(),
          :eob => integer(),
          :eom => integer()
        }

  @doc """
  Retrieves the robot status of the Mecademic Robot.
  """
  def get_status_robot(pid) do
    pid
    |> run_command("GetStatusRobot")
    |> then(&Enum.zip(@robot_status_keys, &1))
    |> Enum.into(%{})
  end

  @spec get_status_gripper(pid()) :: %{
          :gripper_enabled => integer(),
          :homing_state => integer(),
          :holding_part => integer(),
          :limit_reached => integer(),
          :error_state => integer(),
          :force_overload => integer()
        }

  @doc """
  Retrieves the gripper status of the Mecademic Robot.
  """
  def get_status_gripper(pid) do
    pid
    |> run_command("GetStatusGripper")
    |> then(&Enum.zip(@gripper_status_keys, &1))
    |> Enum.into(%{})
  end

  @spec in_error_mode?(pid()) :: boolean()

  @doc """
  Status method that checks whether the Mecademic Robot is in error mode.
  """
  def in_error_mode?(pid) do
    :sys.get_state(pid) |> Map.get(:error_mode)
  end

  @spec set_eob(pid(), integer()) :: command_response()

  @doc """
  Sets End of Block answer active or inactive in the Mecademic Robot. Parameter `e`
  enables (1) EOB or disables (0) EOB.
  """
  def set_eob(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eob: e} end)
    pid |> run_command("SetEOB", [e])
  end

  @spec set_eom(pid(), integer()) :: command_response()

  @doc """
  Sets End of Movement answer active or inactive in the Mecademic Robot. Parameter `e`
  enables (1) EOM or disables (0) EOM.
  """
  def set_eom(pid, e) do
    :sys.replace_state(pid, fn state -> %{state | eom: e} end)
    pid |> run_command("SetEOM", [e])
  end

  @spec reset_error(pid()) :: command_response()

  @doc """
  Resets the error in the Mecademic Robot.
  """
  def reset_error(pid) do
    pid
    |> run_command("ResetError")
    |> tap(fn response ->
      # set the error_mode state to false if the response suggests the error was reset
      :sys.replace_state(pid, fn state ->
        %{state | error_mode: !Enum.member?(@reset_error_responses, response)}
      end)
    end)
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
        %{state | queueing: true, user_eom: eom}
      end)

      set_eom(pid, 0)
    else
      user_eom = :sys.get_state(pid) |> Map.get(:user_eom)

      :sys.replace_state(pid, fn state ->
        %{state | queueing: false}
      end)

      set_eom(pid, user_eom)
    end

    :sys.get_state(pid) |> Map.get(:queueing)
  end

  @spec delay(pid(), float()) :: command_response()

  @doc """
  Gives the Mecademic Robot a wait time before performing another action.
  """
  def delay(pid, t) do
    pid |> run_command("Delay", [t])
  end

  @spec move_joints(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Moves the joints of the Mecademic Robot to the desired angles. Each theta argument corresponds
  to a joint number.
  """
  def move_joints(pid, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6) do
    pid |> run_command("MoveJoints", [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
  end

  @spec move_lin(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Moves the Mecademic Robot tool reference in a straight line to final
  point with specified direction.
  """
  def move_lin(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("MoveLin", [x, y, z, alpha, beta, gamma])
  end

  @spec move_lin_rel_trf(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Moves the Mecademic Robot tool reference frame to specified coordinates and heading.
  """
  def move_lin_rel_trf(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("MoveLinRelTRF", [x, y, z, alpha, beta, gamma])
  end

  @spec move_lin_rel_wrf(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Moves the Mecademic Robot world reference frame to specified coordinates and heading.
  """
  def move_lin_rel_wrf(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("MoveLinRelWRF", [x, y, z, alpha, beta, gamma])
  end

  @spec move_pose(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Moves the Mecademic Robot joints to have the TRF at (x, y, z) with heading (alpha, beta, gamma).
  """
  def move_pose(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("MovePose", [x, y, z, alpha, beta, gamma])
  end

  @spec set_blending(pid(), float()) :: command_response()

  @doc """
  Sets the blending of the Mecademic Robot. Parameter `p` enables between `1` and `100` and
  disables at `0`.
  """
  def set_blending(pid, p) do
    pid |> run_command("SetBlending", [p])
  end

  @spec set_auto_conf(pid(), integer()) :: command_response()

  @doc """
  Enables or Disables the automatic robot configuration selection and has effect only on
  the MovePose command. Parameter `e`  enables (1) EOB or disables (0).
  """
  def set_auto_conf(pid, e) do
    pid |> run_command("SetAutoConf", [e])
  end

  @spec set_cart_acc(pid(), float()) :: command_response()

  @doc """
  Sets the cartesian accelerations of the linear and angular movements of the
  Mecademic Robot end effector. Parameter `p` should be between `1` and `100`.
  """
  def set_cart_acc(pid, p) do
    pid |> run_command("SetCartAcc", [p])
  end

  @spec set_cart_ang_vel(pid(), float()) :: command_response()

  @doc """
  Sets the cartesian angular velocity of the Mecademic Robot TRF with respect to its WRF.
  Parameter `w` should be between `0.001` and `180`.
  """
  def set_cart_ang_vel(pid, w) do
    pid |> run_command("SetCartAngVel", [w])
  end

  @spec set_cart_lin_vel(pid(), float()) :: command_response()

  @doc """
  Sets the cartesian linear velocity of the Mecademic Robot's TRF relative to its WRF.
  Parameter `v` should be between `0.001` and `500`.
  """
  def set_cart_lin_vel(pid, v) do
    pid |> run_command("SetCartLinVel", [v])
  end

  @spec set_conf(pid(), integer(), integer(), integer()) :: command_response()

  @doc """
  Sets the desired Mecademic Robot inverse kinematic configuration to be observed in
  the MovePose command. Parameters `c1`, `c3`, and `c5` should be either `1` or `-1`.
  """
  def set_conf(pid, c1, c3, c5) do
    pid |> run_command("SetConf", [c1, c3, c5])
  end

  @spec set_gripper_force(pid(), float()) :: command_response()

  @doc """
  Sets the Gripper's grip force. Parameter `p` should be between `1` and `100`.
  """
  def set_gripper_force(pid, p) do
    pid |> run_command("SetGripperForce", [p])
  end

  @spec set_gripper_vel(pid(), float()) :: command_response()

  @doc """
  Sets the Gripper fingers' velocity with respect to the gripper. Parameter `p` should be
  between `1` and `100`.
  """
  def set_gripper_vel(pid, p) do
    pid |> run_command("SetGripperVel", [p])
  end

  @spec set_joint_acc(pid(), float()) :: command_response()

  @doc """
  Sets the acceleration of the joints. Parameter `p` should be between `1` and `100`.
  """
  def set_joint_acc(pid, p) do
    pid |> run_command("SetJointAcc", [p])
  end

  @spec set_joint_vel(pid(), float()) :: command_response()

  @doc """
  Sets the angular velocities of the Mecademic Robot's joints. `velocity` should be
  between `1` and `100`.
  """
  def set_joint_vel(pid, velocity) do
    pid |> run_command("SetJointVel", [velocity])
  end

  @spec set_trf(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Sets the Mecademic Robot TRF at (x, y, z) and heading (alpha, beta, gamma) with respect
  to the FRF.
  """
  def set_trf(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("SetTRF", [x, y, z, alpha, beta, gamma])
  end

  @spec set_wrf(pid(), float(), float(), float(), float(), float(), float()) ::
          command_response()

  @doc """
  Sets the Mecademic Robot WRF at (x, y, z) and heading (alpha, beta, gamma) with respect
  to the BRF.
  """
  def set_wrf(pid, x, y, z, alpha, beta, gamma) do
    pid |> run_command("SetWRF", [x, y, z, alpha, beta, gamma])
  end

  @spec build_command(String.t(), list(integer() | float() | String.t()) | nil) ::
          command_response()

  @doc """
  Builds the command string to send to the Mecademic Robot from the function name and
  arguments the command needs.
  """
  def build_command(command, nil), do: command

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

  @doc """
  Parses the raw response into the response code and body.
  """
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

  @doc """
  Decodes the response body into float values, integer values, or a string depending
  on the response code.
  """
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

  @spec error_code?(integer()) :: boolean()

  @doc """
  Returns whether or not the response code is an error code.
  This encompasses the 1000-1999 command errors, and general errors in the 3000 range.
  """
  def error_code?(code) when code in 1000..1999, do: true

  def error_code?(code) when code in [3001, 3003, 3005, 3009, 3014, 3026], do: true

  def error_code?(_), do: false
end
