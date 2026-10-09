using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Media;

namespace Jump_Bruteforcer
{
    internal static class SearchOutput
    {
        /// <summary>
        /// For a given PlayerNode, returns the inputs to get there and the path taken through the game space
        /// For a given PlayerNode, writes to a file the states of all nodes on the path through the game space ending at the current node
        /// </summary>
        /// <returns>a tuple containing the list of inputs and a PointCollection representing the path</returns>
        public static (List<Input> Inputs, PointCollection Points) GetPath(PlayerNode root, int endNode, PathLinkStore pathLinks, CollisionMap collisionMap)
        {
            List<Input> inputs = new List<Input>();
            List<Point> points = new List<Point>();
            List<PlayerNode> path = new List<PlayerNode>();
            int currentNodeIndex = endNode;
            Input currentInput;


            while (currentNodeIndex != 0)
            {
                PathLink link = pathLinks[currentNodeIndex];
                currentInput = link.Input;
                inputs.Add(currentInput);
                currentNodeIndex = link.ParentIndex;
            }
            inputs.Reverse();
            PlayerNode curr = root;
            points.Add(new Point(curr.State.X, curr.State.RoundedY));
            path.Add(curr);
            foreach (Input input in inputs)
            {
                curr = curr.NewState(input, collisionMap);
                points.Add(new Point(curr.State.X, curr.State.RoundedY));
                path.Add(curr);
            }
            string states = string.Join<PlayerNode>("\n", path.ToArray());
            string outputPath = Environment.GetEnvironmentVariable("JUMP_STATE_OUTPUT_DIRECTORY")
                ?? Path.Join(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Jump Bruteforcer macros");
            Directory.CreateDirectory(outputPath);
            File.WriteAllText(Path.Join(outputPath, $"states.txt"), states);
            return (inputs, new PointCollection(points));
        }

        /// <summary>
        /// Replays an input sequence from the root and returns the drawn path. Used by the layered
        /// search, which reconstructs its input list from parent links instead of a node index.
        /// </summary>
        public static PointCollection GetPathPoints(PlayerNode root, List<Input> inputs, CollisionMap collisionMap)
        {
            List<Point> points = new() { new Point(root.State.X, root.State.RoundedY) };
            PlayerNode cur = root;
            foreach (Input input in inputs)
            {
                PlayerNode? next = cur.NewState(input, collisionMap);
                if (next is null) break;
                cur = next;
                points.Add(new Point(cur.State.X, cur.State.RoundedY));
            }
            return new PointCollection(points);
        }

        /// <summary>
        /// Human readable name of the inputs of a single frame. The A/D nudge bits are printed as
        /// the keys they stand for, the other bits keep their enum names.
        /// </summary>
        public static string GetInputName(Input input)
        {
            if (input == Input.Neutral)
            {
                return nameof(Input.Neutral);
            }

            StringBuilder sb = new();
            AppendFlag(sb, input, Input.Left, nameof(Input.Left));
            AppendFlag(sb, input, Input.Right, nameof(Input.Right));
            AppendFlag(sb, input, Input.Jump, nameof(Input.Jump));
            AppendFlag(sb, input, Input.Release, nameof(Input.Release));
            AppendFlag(sb, input, Input.NudgeLeft, "A");
            AppendFlag(sb, input, Input.NudgeRight, "D");
            return sb.ToString();
        }

        private static void AppendFlag(StringBuilder sb, Input input, Input flag, string name)
        {
            if ((input & flag) == flag)
            {
                if (sb.Length > 0)
                {
                    sb.Append(", ");
                }
                sb.Append(name);
            }
        }

        public static string GetInputString(List<Input> inputs)
        {
            if (inputs.Count == 0)
                // return "Frames: 0";
                return "";

            StringBuilder sb = new();

            //sb.AppendLine($"Frames: {inputs.Count}");

            Input PreviousInput = inputs[0];
            int Count = 1;

            for (int i = 1; i < inputs.Count; i++)
            {
                if (inputs[i] == PreviousInput)
                {
                    Count++;
                }
                else
                {
                    sb.AppendLine($"{GetInputName(PreviousInput)}{(Count > 1 ? $" x{Count}" : "")}");
                    PreviousInput = inputs[i];
                    Count = 1;
                }
            }

            sb.AppendLine($"{GetInputName(PreviousInput)}{(Count > 1 ? $" x{Count}" : "")}");

            return sb.ToString();
        }

        // https://github.com/namelessiw/old-bruteforcer-rewrite-5/blob/40e494137445f5159e853f06a178ca44ba67dda5/Player.cs#L344
        public static string GetVerticalInputString(List<Input> inputs, bool oneframeConvention)
        {
            if (inputs.Count == 0)
            {
                return "";
            }

            StringBuilder sb = new();
            int frame = 0;
            bool released = true; // dependent on starting vspeed to distinguish fixed vspeed jump and walkoff?

            /*
            1f 3p 4f 12p
            3p 3f 3p
            0f+1+1+1 10p
            3f 0p 5f 14p
            */

            foreach (Input input in inputs)
            {
                if ((input & Input.Jump) == Input.Jump)
                {
                    if (frame > 0)
                    {
                        if (released)
                        {
                            sb.Append($" {frame}p");
                        }
                        else
                        {
                            sb.Append($" {(oneframeConvention ? frame + 1 : frame)}f 0p");
                        }
                    }

                    released = false;
                    frame = 0;
                }
                if ((input & Input.Release) == Input.Release)
                {
                    if (released)
                    {
                        sb.Append($"+{frame}");
                    }
                    else
                    {
                        sb.Append($" {(oneframeConvention ? frame + 1 : frame)}f");
                    }

                    released = true;
                    frame = 0;
                }

                frame++;
            }

            if (released)
            {
                sb.Append($" {frame}p");
            }
            else
            {
                sb.Append($" {(oneframeConvention ? frame + 1 : frame)}f");
            }

            return sb.ToString().Trim();
        }
        /// <summary>
        /// Everything that moves the kid horizontally: the arrow keys and the A/D nudge.
        /// </summary>
        private const Input HorizontalMask = Input.Left | Input.Right | Input.NudgeLeft | Input.NudgeRight;

        /// <summary>
        /// One token of the horizontal input string: the arrow keys as L/R/LR, the A/D nudge as
        /// A/D (they are written after the arrows when a frame uses both), neutral as p.
        /// </summary>
        private static string GetHorizontalToken(Input horizontalInput)
        {
            if (horizontalInput == Input.Neutral)
            {
                return "p";
            }

            StringBuilder sb = new(4);
            if ((horizontalInput & Input.Left) == Input.Left)
            {
                sb.Append('L');
            }
            if ((horizontalInput & Input.Right) == Input.Right)
            {
                sb.Append('R');
            }
            if ((horizontalInput & Input.NudgeLeft) == Input.NudgeLeft)
            {
                sb.Append('A');
            }
            if ((horizontalInput & Input.NudgeRight) == Input.NudgeRight)
            {
                sb.Append('D');
            }
            return sb.ToString();
        }

        public static string GetHorizontalInputString(List<Input> inputs)
        {
            if (inputs.Count == 0)
            {
                return "";
            }

            StringBuilder sb = new();
            int frame = 0;

            /*
            1l 1p 1r 1lr 1d
            */

            Input lastInput = inputs[0] & HorizontalMask;

            foreach (Input input in inputs)
            {
                Input horizontalInput = input & HorizontalMask;

                if (horizontalInput != lastInput)
                {
                    sb.Append($"{frame}{GetHorizontalToken(lastInput)} ");
                    frame = 0;
                }

                lastInput = horizontalInput;
                frame++;
            }

            sb.Append($"{frame}{GetHorizontalToken(lastInput)} ");

            return sb.ToString().Trim();
        }

        public static string GetMacro(List<Input> inputs)
        {
            if (inputs.Count == 0)
                return "";

            StringBuilder sb = new StringBuilder();

            Input Direction = Input.Neutral, NextDirection;

            foreach (Input input in inputs)
            {
                bool InputChanged = false;

                NextDirection = input & Input.Right | input & Input.Left;

                if (Direction != NextDirection)
                {
                    if (Direction != Input.Neutral)
                    {
                        sb.Append((Direction == Input.Right ? "RightArrow" : "LeftArrow") + "(R)");
                        InputChanged = true;
                    }

                    if (NextDirection != Input.Neutral)
                    {
                        sb.Append((InputChanged ? "," : "") + (NextDirection == Input.Right ? "RightArrow" : "LeftArrow") + "(PRP)");
                    }

                    InputChanged = true;
                }
                else if (Direction != Input.Neutral)
                {
                    sb.Append((Direction == Input.Right ? "RightArrow" : "LeftArrow") + "(RP)");
                    InputChanged = true;
                }

                if ((input & Input.Jump) == Input.Jump)
                {
                    sb.Append((InputChanged ? "," : "") + "J(PR)");

                    InputChanged = true;
                }
                if ((input & Input.Release) == Input.Release)
                {
                    sb.Append((InputChanged ? "," : "") + "K(PR)");

                    InputChanged = true;
                }
                // A/D are edge triggered, so one tap per frame: (PR) is press then release inside
                // the frame, which raises keyboard_check_pressed and leaves the key neutral again.
                if ((input & Input.NudgeLeft) == Input.NudgeLeft)
                {
                    sb.Append((InputChanged ? "," : "") + "A(PR)");

                    InputChanged = true;
                }
                if ((input & Input.NudgeRight) == Input.NudgeRight)
                {
                    sb.Append((InputChanged ? "," : "") + "D(PR)");

                    InputChanged = true;
                }

                Direction = NextDirection;

                sb.Append('>');
            }

            return sb.ToString();
        }
    }
}
