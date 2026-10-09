using FluentAssertions;
using Jump_Bruteforcer;

namespace TestBrute
{
    public class TestSearchOutput
    {
        [Fact]
        public void TestMacroTapsTheNudgeKeys()
        {
            // A/D are edge triggered, so (PR) - press then release inside the frame - is what
            // raises keyboard_check_pressed while leaving the key neutral again.
            SearchOutput.GetMacro(new() { Input.NudgeRight }).Should().Be("D(PR)>");
            SearchOutput.GetMacro(new() { Input.NudgeLeft }).Should().Be("A(PR)>");

            // one tap per frame, so holding A for two frames is two presses
            SearchOutput.GetMacro(new() { Input.NudgeLeft, Input.NudgeLeft }).Should().Be("A(PR)>A(PR)>");
        }

        [Fact]
        public void TestMacroSeparatesTheNudgeFromTheOtherKeys()
        {
            // keys of the same frame are comma separated, and the frame ends with '>'
            SearchOutput.GetMacro(new() { Input.Left | Input.Jump | Input.NudgeRight })
                .Should().Be("LeftArrow(PRP),J(PR),D(PR)>");
            SearchOutput.GetMacro(new() { Input.Right | Input.NudgeRight })
                .Should().Be("RightArrow(PRP),D(PR)>");
            SearchOutput.GetMacro(new() { Input.Release | Input.NudgeLeft })
                .Should().Be("K(PR),A(PR)>");
        }

        [Fact]
        public void TestMacroIsUnchangedWithoutTheNudge()
        {
            // regression: sequences without A/D must produce exactly what they produced before
            SearchOutput.GetMacro(new() { Input.Left, Input.Left | Input.Jump, Input.Neutral, Input.Right, Input.Neutral })
                .Should().Be("LeftArrow(PRP)>LeftArrow(RP),J(PR)>LeftArrow(R)>RightArrow(PRP)>RightArrow(R)>");
            SearchOutput.GetMacro(new() { Input.Jump, Input.Release, Input.Jump })
                .Should().Be("J(PR)>K(PR)>J(PR)>");
            SearchOutput.GetMacro(new()).Should().Be("");
        }

        [Fact]
        public void TestInputStringNamesTheNudgeKeys()
        {
            SearchOutput.GetInputString(new() { Input.Neutral, Input.Neutral, Input.NudgeRight, Input.Neutral })
                .Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries)
                .Should().Equal("Neutral x2", "D", "Neutral");

            SearchOutput.GetInputString(new() { Input.Left | Input.NudgeLeft })
                .Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries)
                .Should().Equal("Left, A");

            // the other bits keep their enum names, in enum order
            SearchOutput.GetInputString(new() { Input.Jump | Input.NudgeLeft | Input.NudgeRight })
                .Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries)
                .Should().Equal("Jump, A, D");

            SearchOutput.GetInputString(new()).Should().Be("");
        }

        [Fact]
        public void TestHorizontalStringShowsTheNudge()
        {
            // a nudge frame is its own horizontal state, it is no longer folded into the pauses
            SearchOutput.GetHorizontalInputString(new() { Input.Neutral, Input.NudgeRight, Input.Neutral, Input.Left, Input.Left, Input.Neutral })
                .Should().Be("1p 1D 1p 2L 1p");
            SearchOutput.GetHorizontalInputString(new() { Input.NudgeLeft, Input.Right | Input.NudgeRight })
                .Should().Be("1A 1RD");

            // regression: unchanged for sequences without A/D
            SearchOutput.GetHorizontalInputString(new() { Input.Left, Input.Right, Input.Neutral, Input.Left | Input.Right })
                .Should().Be("1L 1R 1p 1LR");
            SearchOutput.GetHorizontalInputString(new()).Should().Be("");
        }
    }
}
