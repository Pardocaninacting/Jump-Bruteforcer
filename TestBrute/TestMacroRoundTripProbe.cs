using FluentAssertions;
using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// Does the exported macro mean the same thing as the input list it was written from? The
    /// per frame notation in the strategy text is generated straight from that list, so comparing
    /// it with the macro parsed back is a round trip check of the exporter.
    /// </summary>
    public class TestMacroRoundTripProbe
    {
        private readonly ITestOutputHelper output;

        public TestMacroRoundTripProbe(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        /// <summary>
        /// What the macro says has to be exactly what the search simulated, otherwise the exported
        /// TAS replays something else than the path the user was shown.
        /// </summary>
        [Theory]
        [InlineData(100, 100, 100, 500, "tomo")]                 // a pure fall, razor thin timing
        [InlineData(410, 407.4, 476, 343, "tomo")]               // an ordinary screen
        [InlineData(401, 407.4, 687, 211, "gate")]               // finishes on a warp
        [InlineData(388, 407.4, 541, 407, "platform_invert")]    // rides platforms
        public void TestMacroReproducesTheSolution(int sx, double sy, int gx, int gy, string name)
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{name}.jmap"));
            Search search = new((sx, sy), (gx, gy), map.CollisionMap) { UseLayeredBfs = true };
            SearchResult result = search.RunAStar();
            result.Success.Should().BeTrue();

            List<Input> notation = ParseNotation(result.InputString);
            List<Input> macro = ParseMacro(result.Macro);
            macro.Count.Should().Be(notation.Count);
            for (int i = 0; i < notation.Count; i++)
            {
                macro[i].Should().Be(notation[i], $"macro frame {i}");
            }
            End(map, (sx, sy, gx, gy, name), macro).Goal.Should().BeTrue();
        }

        [Fact]
        [Trait("Category", "Bench")]
        public void ProbeMacroRoundTrip()
        {
            output.WriteLine("map,frames,notationFrames,macroFrames,differingFrames,macroStillFinishes,macroEndsAtGoal");
            foreach (var c in new (int x, double y, int gx, int gy, string name)[]
            {
                (100, 100, 100, 500, "tomo"),
                (410, 407.4, 476, 343, "tomo"),
                (452, 407.4, 482, 343, "minif"),
                (410, 407.4, 485, 407, "co"),
                (410, 407.4, 491, 407, "double"),
                (410, 407.4, 551, 407, "65"),
                (420, 407.4, 477, 375, "ground_dplane"),
                (388, 407.4, 541, 407, "platform_invert"),
                (410, 407.4, 443, 311, "leehe"),
                (399, 487.4, 399, 295, "platform_teleport"),
                (401, 407.4, 687, 211, "gate"),
            })
            {
                Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{c.name}.jmap"));
                Search search = new((c.x, c.y), (c.gx, c.gy), map.CollisionMap) { UseLayeredBfs = true };
                SearchResult result = search.RunAStar();
                if (!result.Success)
                {
                    output.WriteLine($"{c.name},FAIL,,-,-,-,-");
                    continue;
                }

                List<Input> notation = ParseNotation(result.InputString);
                List<Input> macro = ParseMacro(result.Macro);
                int differ = 0;
                for (int i = 0; i < Math.Min(notation.Count, macro.Count); i++)
                {
                    if (notation[i] != macro[i]) differ++;
                }

                (int x, int y, bool goal) = End(map, c, macro);
                output.WriteLine($"{c.name},{notation.Count},{notation.Count},{macro.Count},{differ}," +
                                 $"{End(map, c, macro).Goal},{End(map, c, notation).Goal} ends ({x},{y})");
            }
        }

        private static (int X, int Y, bool Goal) End(Map map, (int x, double y, int gx, int gy, string name) c, List<Input> inputs)
        {
            PlayerNode node = new(c.x, c.y, 0);
            foreach (Input input in inputs)
            {
                PlayerNode? next = node.NewState(input, map.CollisionMap);
                if (next is null) return (0, 0, false);
                node = next;
            }
            bool goal = (Math.Abs(node.State.X - c.gx) <= 1 && node.State.RoundedY == c.gy)
                || map.CollisionMap.onWarp(node.State.X, node.State.RoundedY);
            return (node.State.X, node.State.RoundedY, goal);
        }

        /// <summary>The per frame list the strategy text prints, taken back to inputs.</summary>
        private static List<Input> ParseNotation(string strat)
        {
            var inputs = new List<Input>();
            int start = strat.IndexOf("Inputs per frame:");
            if (start < 0) return inputs;
            foreach (string raw in strat[(start + "Inputs per frame:".Length)..].Split('\n'))
            {
                string line = raw.Trim();
                if (line.Length == 0) continue;
                int repeat = 1;
                int times = line.LastIndexOf(" x", StringComparison.Ordinal);
                if (times > 0 && int.TryParse(line[(times + 2)..], out int parsed))
                {
                    repeat = parsed;
                    line = line[..times];
                }
                Input input = Input.Neutral;
                foreach (string name in line.Split(',', StringSplitOptions.RemoveEmptyEntries))
                {
                    input |= name.Trim() switch
                    {
                        "Left" => Input.Left,
                        "Right" => Input.Right,
                        "Jump" => Input.Jump,
                        "Release" => Input.Release,
                        "A" => Input.NudgeLeft,
                        "D" => Input.NudgeRight,
                        _ => Input.Neutral,
                    };
                }
                for (int i = 0; i < repeat; i++) inputs.Add(input);
            }
            return inputs;
        }

        private static List<Input> ParseMacro(string macro)
        {
            var frames = new List<Input>();
            string[] segments = macro.Split('>');
            int count = segments.Length;
            if (count > 0 && segments[count - 1].Length == 0) count--;
            Input previous = Input.Neutral;
            for (int i = 0; i < count; i++)
            {
                if (segments[i].Length == 0)
                {
                    // An empty frame only means "nothing changed". The direction keys are the only
                    // ones the exporter keeps held across frames; jump, release and the nudges are
                    // written as taps, so they must not be carried over.
                    frames.Add(previous & (Input.Left | Input.Right));
                    continue;
                }
                Input input = Input.Neutral;
                foreach (string token in segments[i].Split(',', StringSplitOptions.RemoveEmptyEntries))
                {
                    if (token.StartsWith("LeftArrow") && !token.Contains("(R)")) input |= Input.Left;
                    else if (token.StartsWith("RightArrow") && !token.Contains("(R)")) input |= Input.Right;
                    else if (token.StartsWith("J(")) input |= Input.Jump;
                    else if (token.StartsWith("K(")) input |= Input.Release;
                    else if (token.StartsWith("A(")) input |= Input.NudgeLeft;
                    else if (token.StartsWith("D(")) input |= Input.NudgeRight;
                }
                previous = input;
                frames.Add(input);
            }
            return frames;
        }
    }
}
