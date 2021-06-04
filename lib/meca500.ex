defmodule Meca500 do
  use GenServer

  @initial_state %{socket: nil}

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

  # def xyz do
  #   {:ok, pid} = start_link()
  #   command(pid, "hello\0")
  # end

  def command(pid, cmd) do
    GenServer.call(pid, {:command, cmd})
  end

  def start_link do
    GenServer.start_link(__MODULE__, @initial_state)
  end

  def init(state) do
    opts = [:binary, packet: :line, line_delimiter: 0, buffer: 1024, active: false]
    {:ok, socket} = :gen_tcp.connect('localhost', 10000, opts)
    {:ok, %{state | socket: socket}}
  end

  def handle_call({:command, cmd}, _, %{socket: socket} = state) do
    :ok = :gen_tcp.send(socket, encode(cmd))

    {:ok, resp} = :gen_tcp.recv(socket, 0)

    {:reply, resp, state}
  end

  def encode(msg) do
    msg
  end

  def decode(msg) do
    msg
  end

  def build_command(command, args) do
    case Enum.count(args) do
      0 ->
        command

      _ ->
        args_list = Enum.join(args, ",")
        "#{command}(#{args_list})"
    end
  end

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
  def answer_codes("PauseMotion", %{eom: 1} = state), do: [2042, 3004]
  def answer_codes("PauseMotion", _), do: [2042]
  def answer_codes("ResetError", _), do: [2005, 2006]
  def answer_codes("ResumeMotion", _), do: [2043]
  def answer_codes("SetEOB", _), do: [2054, 2055]
  def answer_codes("SetEOM", _), do: [2052, 2053]

  def answer_codes(command, %{eob: eob, eom: eom} = state) do
    []
    |> append_eob_answer_code(eob)
    |> append_eom_answer_code(command, eom)
  end

  def answer_codes(_, _), do: []

  defp append_eob_answer_code(list, 1), do: [3012 | list]
  defp append_eob_answer_code(list, _), do: list

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
