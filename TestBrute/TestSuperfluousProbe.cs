using FluentAssertions;
using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// How much of an emitted solution is horizontal input that is not needed at all? Each solution
    /// is replayed with every Left/Right bit removed; if that still finishes the screen, the
    /// directions were free and the notation could have been shorter.
    /// </summary>
    public class TestSuperfluousProbe
    {
        private readonly ITestOutputHelper output;

        public TestSuperfluousProbe(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        [Fact]
        public void ProbeSuperfluousHorizontal()
        {
            output.WriteLine("map,frames,runs,runsWithoutDirections,stillFinishes");
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
            })
            {
                Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\{c.name}.jmap"));
                Search search = new((c.x, c.y), (c.gx, c.gy), map.CollisionMap) { UseLayeredBfs = true };
                SearchResult result = search.RunAStar();
                if (!result.Success)
                {
                    output.WriteLine($"{c.name},FAIL,,-,-");
                    continue;
                }

                List<Input> found = Parse(result.Macro);
                List<Input> stripped = found.Select(i => i & ~(Input.Left | Input.Right)).ToList();
                bool finishes = Reaches(map, c, stripped);
                output.WriteLine($"{c.name},{found.Count},{Runs(found)},{Runs(stripped)},{finishes}");
            }
        }

        /// <summary>tomo (100,100) -> (100,500): the fall alone never lands on the goal row.</summary>
        [Fact]
        public void TestFallNeedsAPhaseChange()
        {
            Map map = Parser.Parse(File.ReadAllText(@$"..\..\..\jmaps\tomo.jmap"));

            // falling straight down never lands on RoundedY == 500: the frame sampling skips it
            PlayerNode node = new(100, 100, 0);
            bool landed = false;
            for (int frame = 0; frame < 200; frame++)
            {
                node = node.NewState(Input.Neutral, map.CollisionMap)!;
                if (node.State.RoundedY == 500) landed = true;
            }
            landed.Should().BeFalse("the fall has a fixed phase that steps over the goal row");

            // the emitted solution reaches it, and the directions it uses are not needed for that
            Search search = new((100, 100), (100, 500), map.CollisionMap) { UseLayeredBfs = true };
            SearchResult result = search.RunAStar();
            result.Success.Should().BeTrue();
            List<Input> found = Parse(result.Macro);
            List<Input> stripped = found.Select(i => i & ~(Input.Left | Input.Right)).ToList();
            Reaches(map, (100, 100, 100, 500, "tomo"), stripped).Should().BeTrue();
            found.Count.Should().Be(58);
            Runs(found).Should().BeLessThanOrEqualTo(4, "the search drops the direction input the fall does not need");
            Runs(stripped).Should().BeLessThanOrEqualTo(Runs(found));
        }

        private static bool Reaches(Map map, (int x, double y, int gx, int gy, string name) c, List<Input> inputs)
        {
            PlayerNode node = new(c.x, c.y, 0);
            foreach (Input input in inputs)
            {
                PlayerNode? next = node.NewState(input, map.CollisionMap);
                if (next is null) return false;
                node = next;
            }
            return Math.Abs(node.State.X - c.gx) <= 1 && node.State.RoundedY == c.gy;
        }

        private static int Runs(List<Input> inputs)
        {
            int runs = 0;
            for (int i = 0; i < inputs.Count; i++)
            {
                if (i == 0 || inputs[i] != inputs[i - 1]) runs++;
            }
            return runs;
        }

        private static List<Input> Parse(string macro)
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
                    frames.Add(previous & (Input.Left | Input.Right));  // taps are not carried over
                    continue;
                }
                Input input = Input.Neutral;
                foreach (string token in segments[i].Split(',', StringSplitOptions.RemoveEmptyEntries))
                {
                    if (token.StartsWith("LeftArrow") && !token.Contains("(R)")) input |= Input.Left;
                    else if (token.StartsWith("RightArrow") && !token.Contains("(R)")) input |= Input.Right;
                    else if (token.StartsWith("J(")) input |= Input.Jump;
                    else if (token.StartsWith("K(")) input |= Input.Release;
                }
                previous = input;
                frames.Add(input);
            }
            return frames;
        }
    }
}
