using System.Windows.Media;
using System.Text.Json;
using System.Collections.Immutable;
using System.Windows;
using System.Runtime.InteropServices;
using System.Diagnostics;

namespace Jump_Bruteforcer
{
    [Flags]
    public enum Bools: byte
    {
        None = 0,
        CanDJump = 1,
        OnPlatform = 2,
        FacingRight = 4,
        InvertedGravity = 8,
        ParentInvertedGravity = 16,
        // Search only flag: the jump key is currently up. It is maintained by the
        // search when it is asked to avoid the cactus technique, and left at None
        // by everything else.
        JumpReleased = 32
    }
    [StructLayout(LayoutKind.Auto)]
    public readonly record struct State 
    {

        public int X { get; init; }
        public double Y { get; init; }
        public double VSpeed { get; init; }
        public Bools Flags { get; init; }
        public int RoundedY { get { return (int)Math.Round(Y); } }




    }

    public readonly record struct NeighborCandidate(State State, Input Input, ulong Key);

    public class PlayerNode : IEquatable<PlayerNode>
    {
        public static bool ShareHorizontal = true;
        public static bool DisableFacingNormalization = false;
        const int epsilon = 10;
        public const int MaxNeighborCount = 12;
        public State State { get; set; }
        public int NodeIndex { get; set; }
        public uint PathCost { get; set; }

        public static readonly ImmutableArray<Input> inputs = ImmutableArray.Create(Input.Neutral, Input.Left, Input.Right);
        public static readonly ImmutableArray<Input> inputsJump = ImmutableArray.Create(Input.Jump, Input.Left | Input.Jump, Input.Right | Input.Jump, Input.Jump | Input.Release, Input.Left | Input.Jump | Input.Release, Input.Right | Input.Jump | Input.Release);
        public static readonly ImmutableArray<Input> inputsRelease = ImmutableArray.Create(Input.Release, Input.Left | Input.Release, Input.Right | Input.Release);
        public PlayerNode(int x, double y, double vSpeed, Bools flags = Bools.CanDJump | Bools.FacingRight, Input? action = null, int nodeIndex = 0) =>
            (State, NodeIndex, PathCost) = (new State() { X = x, Y = y, VSpeed = vSpeed, Flags = flags }, nodeIndex, uint.MaxValue);

        public PlayerNode(State state)
        {
            State = state;
            PathCost = uint.MaxValue;
        }

        public bool IsGoal((int x, int y) goal) => Math.Abs(State.X - goal.x) <= 1 & State.RoundedY == goal.y;








        /// <summary>
        /// creates the set of all unique states that can be reached in one frame from the current state with arbitrary inputs.
        /// states with fewer inputs are favored if two states are the same. States inside playerkillers are excluded.
        /// When <paramref name="trackJumpReleased"/> is set, the jump key state is carried in the state itself and a
        /// Release input is only generated while the key is still held, which forbids the cactus technique.
        /// </summary>
        /// <returns>a Hashset of playerNodes</returns>
        public static int GetNeighborCandidates(State currentState, CollisionMap CollisionMap, NeighborCandidate[] neighbors, bool trackJumpReleased = false, bool ignoreFacing = false)
        {
            int neighborCount = 0;
            PlayerUpdateContext updateContext = Player.PrepareUpdateContext(currentState, CollisionMap);
            EmitGroup(CollisionMap, neighbors, Input.Neutral, Input.Left, Input.Right, ref neighborCount);
            //corresponds to global.grav = 1
            bool globalGravInverted = (currentState.Flags & Bools.InvertedGravity) == Bools.InvertedGravity;

            double checkOffset = globalGravInverted ? -1 : 1;
            // Releasing a jump key that is not held is what makes a cactus strat possible.
            bool jumpHeld = !trackJumpReleased || (currentState.Flags & Bools.JumpReleased) == Bools.None;
            if (jumpHeld && Math.Sign(currentState.VSpeed) == -checkOffset)
            {
                EmitGroup(CollisionMap, neighbors, Input.Release, Input.Left | Input.Release, Input.Right | Input.Release, ref neighborCount);
            }

            // When Jump cannot mutate this state, all six Jump variants produce
            // states already emitted by the normal or Release groups above.
            if (Player.JumpCanChangeState(currentState, updateContext))
            {
                EmitGroup(CollisionMap, neighbors, Input.Jump, Input.Left | Input.Jump, Input.Right | Input.Jump, ref neighborCount);
                EmitGroup(CollisionMap, neighbors, Input.Jump | Input.Release, Input.Left | Input.Jump | Input.Release, Input.Right | Input.Jump | Input.Release, ref neighborCount);
            }

            return neighborCount;

            /// <summary>
            /// The three horizontal choices of one vertical input group only differ by x, the facing
            /// bit and the collision at the destination. When the middle frame neither collided nor
            /// touched a vine, and the side destination pixels are free, the side variants are derived
            /// from the middle result instead of being simulated again.
            /// </summary>
            void EmitGroup(CollisionMap collisionMap, NeighborCandidate[] candidates, Input middle, Input left, Input right, ref int count)
            {
                State? middleState = Player.Update(currentState, middle, collisionMap, updateContext, out bool middleCollided);
                if (middleState is not State middleResult || !Player.IsAlive(middleResult))
                {
                    AddSimulated(collisionMap, candidates, left, ref count);
                    AddSimulated(collisionMap, candidates, right, ref count);
                    return;
                }

                AddCandidate(candidates, middleResult, middle, ref count);

                // After the facing flip the step re-reads the vine distances with the *new*
                // facing, so all four variants have to be clear for the result to be x independent.
                VineDistances vines = updateContext.Vines;
                bool derivable = ShareHorizontal && !middleCollided
                    && middleResult.X == currentState.X
                    && vines.LeftFacingRight == VineDistance.FAR
                    && vines.LeftFacingLeft == VineDistance.FAR
                    && vines.RightFacingRight == VineDistance.FAR
                    && vines.RightFacingLeft == VineDistance.FAR;

                if (derivable && Derive(collisionMap, middleResult, -1, left, out State derivedLeft))
                {
                    AddCandidate(candidates, derivedLeft, left, ref count);
                }
                else
                {
                    AddSimulated(collisionMap, candidates, left, ref count);
                }

                if (derivable && Derive(collisionMap, middleResult, 1, right, out State derivedRight))
                {
                    AddCandidate(candidates, derivedRight, right, ref count);
                }
                else
                {
                    AddSimulated(collisionMap, candidates, right, ref count);
                }
            }

            bool Derive(CollisionMap collisionMap, State middleResult, int sign, Input input, out State derived)
            {
                bool kidUpsidedown = (currentState.Flags & Bools.ParentInvertedGravity) == Bools.ParentInvertedGravity;
                int x = middleResult.X + sign * PhysicsParams.WALKING_SPEED;
                if (collisionMap.GetCollisionTypes(x, middleResult.Y, kidUpsidedown) != CollisionType.None)
                {
                    derived = default;
                    return false;
                }

                Bools flags = sign > 0 ? middleResult.Flags | Bools.FacingRight : middleResult.Flags & ~Bools.FacingRight;
                derived = new State { X = x, Y = middleResult.Y, VSpeed = middleResult.VSpeed, Flags = flags };
                return Player.IsAlive(derived);
            }

            void AddSimulated(CollisionMap collisionMap, NeighborCandidate[] candidates, Input input, ref int count)
            {
                State? nextState = Player.Update(currentState, input, collisionMap, updateContext);
                if (nextState is not State state || !Player.IsAlive(state))
                {
                    return;
                }
                AddCandidate(candidates, state, input, ref count);
            }

            void AddCandidate(NeighborCandidate[] candidates, State state, Input input, ref int count)
            {
                if (trackJumpReleased)
                {
                    state = TrackJumpReleased(state, input);
                }

                ulong key = StateKey(state, ignoreFacing);
                for (int i = 0; i < count; i++)
                {
                    if (candidates[i].Key == key)
                    {
                        return;
                    }
                }

                candidates[count++] = new NeighborCandidate(state, input, key);
            }
        }

        /// <summary>
        /// Applies the jump key transitions of a single frame to the search state. Pressing the key arms exactly one
        /// release and releasing it disarms the key again, which mirrors the accounting used by the input formatter.
        /// </summary>
        private static State TrackJumpReleased(State state, Input input)
        {
            bool released = (state.Flags & Bools.JumpReleased) != Bools.None;
            if ((input & Input.Jump) == Input.Jump)
            {
                released = false;
            }
            if ((input & Input.Release) == Input.Release)
            {
                released = true;
            }

            Bools flags = released ? state.Flags | Bools.JumpReleased : state.Flags & ~Bools.JumpReleased;
            return flags == state.Flags ? state : state with { Flags = flags };
        }

        public IEnumerable<(PlayerNode Node, Input Input, ulong Hash)> GetNeighbors(CollisionMap CollisionMap, bool trackJumpReleased = false, bool ignoreFacing = false)
        {
            var candidates = new NeighborCandidate[MaxNeighborCount];
            int count = GetNeighborCandidates(State, CollisionMap, candidates, trackJumpReleased, ignoreFacing);
            for (int i = 0; i < count; i++)
            {
                NeighborCandidate candidate = candidates[i];
                yield return (new PlayerNode(candidate.State), candidate.Input, candidate.Key);
            }
        }

        /// <summary>
        /// Creates the next state in the tree after applying inputs to the game starting from the current state
        /// </summary>
        /// <param name="input"></param> the inputs for the next frame
        /// <param name="CollisionMap"></param> the game field
        /// <returns>A new PlayerNode that results from running inputs on the collision map</returns>
        public PlayerNode? NewState(Input input, CollisionMap CollisionMap)
        {

            State? newState = Player.Update(State, input, CollisionMap);
            if (newState != null)
            {
                return new PlayerNode(newState.Value);
            }
            return null;
            
        }

        public bool Equals(PlayerNode? other)
        {
            if (other is null)
            {
                return false;
            }

            return State.X == other.State.X & ApproximatelyEquals(State.Y, other.State.Y) &
            ApproximatelyEquals(State.VSpeed, other.State.VSpeed) & State.Flags == other.State.Flags;
        }

        private static double Quantize(double a)
        {
            return Math.Round(a * epsilon);
        }
        private static bool ApproximatelyEquals(double a, double b)
        {
            return Quantize(a) == Quantize(b);
        }

        public override int GetHashCode() => StateKey(State).GetHashCode();
        public ulong Hash() => StateKey(State);

        /// <summary>
        /// The flags that actually influence the simulation. FacingRight is only read by the
        /// vine distance lookups, so a map without vines can leave it out of the key and
        /// merge twice as many states. The real flags are still used by the physics.
        /// </summary>
        public static Bools NormalizeFlags(Bools flags, bool ignoreFacing) =>
            ignoreFacing ? flags & ~Bools.FacingRight : flags;

        public static ulong StateKey(State state) => StateKey(state, ignoreFacing: false);

        public static ulong StateKey(State state, bool ignoreFacing)
        {
            int quantizedY = (int)Quantize(state.Y);
            int quantizedVSpeed = (int)Quantize(state.VSpeed);
            Bools flags = NormalizeFlags(state.Flags, ignoreFacing);

            // Search states are in bounds: X needs 10 bits and Y*10 needs
            // 13 bits. VSpeed keeps its full signed 32-bit representation.
            Debug.Assert((uint)state.X <= 0x3ff);
            Debug.Assert((uint)quantizedY <= 0x1fff);
            return ((ulong)(uint)state.X & 0x3ffUL)
                | (((ulong)(uint)quantizedY & 0x1fffUL) << 10)
                | ((ulong)(uint)quantizedVSpeed << 23)
                | ((ulong)(byte)flags << 55);
        }
        
        public override string ToString() => $"{{State: {JsonSerializer.Serialize(State)}}}";
    }
}
