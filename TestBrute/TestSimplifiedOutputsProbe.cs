using FluentAssertions;
using Jump_Bruteforcer;
using System.Windows;
using Xunit.Abstractions;

namespace TestBrute
{
    /// <summary>
    /// The drawn path, the macro and the candidate list all have to describe the solution the user
    /// is shown, including after the direction input the screen does not need was dropped.
    /// </summary>
    public class TestSimplifiedOutputsProbe
    {
        private readonly ITestOutputHelper output;

        public TestSimplifiedOutputsProbe(ITestOutputHelper output)
        {
            if (Application.Current == null) new Application();
            this.output = output;
        }

        [Fact]
        public void TestPathMatchesTheNotationAfterSimplifying()
        {
            Map map = Parser.Parse(File.ReadAllText(@"..\..\..\jmaps\tomo.jmap"));

            foreach (bool optimal in new[] { false, true })
            {
                Search search = new((100, 100), (100, 500), map.CollisionMap) { UseLayeredBfs = optimal };
                SearchResult result = search.RunAStar();
                result.Success.Should().BeTrue();

                List<Input> notation = ParseNotation(result.InputString);
                notation.Count.Should().Be(58);
                Runs(notation).Should().BeLessThanOrEqualTo(4, "the directions were free input");

                // the drawn path has to be the path of the solution that is printed
                PlayerNode node = new(100, 100, 0);
                int index = 0;
                foreach (Point point in search.PlayerPath)
                {
                    ((int)Math.Round(point.X)).Should().Be(node.State.X, $"path point {index}");
                    ((int)Math.Round(point.Y)).Should().Be(node.State.RoundedY, $"path point {index}");
                    if (index == notation.Count) break;
                    node = node.NewState(notation[index], map.CollisionMap)!;
                    index++;
                }

                foreach (SolutionCandidate candidate in search.Candidates)
                {
                    candidate.Runs.Should().Be(Runs(notation), "the candidate has to describe the same solution");
                }
                output.WriteLine($"optimal={optimal}: {search.Candidates.Count} candidates, " +
                                 $"first {search.Candidates[0].Label}, path points {search.PlayerPath.Count}");
            }
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
    }
}
