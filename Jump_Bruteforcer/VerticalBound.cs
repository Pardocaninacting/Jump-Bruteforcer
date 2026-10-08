namespace Jump_Bruteforcer
{
    /// <summary>
    /// Vertical pattern database: an admissible lower bound on the remaining frames that only looks
    /// at the vertical state (position, speed, double jump availability) and ignores X completely.
    ///
    /// The abstraction is a relaxation of the real movement rules, so its distances can never
    /// overestimate the real ones:
    ///  * a row counts as landable when any column of it is occupied, so the player can always find
    ///    ground where the real map might not offer any at that particular x;
    ///  * ceilings, spikes and water are ignored;
    ///  * the quantised result is rounded to the nearest cell and lookups take the best of the
    ///    neighbouring cells.
    ///
    /// What it does capture, and the geometric flood fill does not, is the actual vertical dynamics:
    /// gravity, jump heights, the release multiplier and whether a double jump is still available.
    /// That is where most of the remaining heuristic slack sits on jump heavy screens.
    /// </summary>
    public static class VerticalBound
    {
        private const int QH = (Map.HEIGHT - 1) * 10 + 1;
        private const int QVCount = 189;
        private const int MinQV = -94;
        private const int MaxQV = MinQV + QVCount - 1;
        private const int StateCount = QH * QVCount * 2;
        private const int MaxStored = ushort.MaxValue;

        private const double Gravity = PhysicsParams.GRAVITY;
        private const double SingleJump = -PhysicsParams.SJUMP_VSPEED;
        private const double DoubleJump = -PhysicsParams.DJUMP_VSPEED;
        private const double MaxFall = PhysicsParams.MAX_VSPEED;

        public static ushort[] Build(CollisionMap map, int goalRow)
        {
            bool[] landable = BuildLandableRows(map);
            bool[] grounded = BuildGroundedRows(landable);

            // Forward edges of the abstract graph, bucketed by target for the reverse sweep.
            int[] counts = new int[StateCount + 1];
            for (int state = 0; state < StateCount; state++)
            {
                Decode(state, out int qy, out int qv, out int dj);
                for (int input = 0; input < 4; input++)
                {
                    if (Step(landable, grounded, qy, qv, dj, input, out int ny, out int nv, out int ndj))
                    {
                        counts[Encode(ny, nv, ndj) + 1]++;
                    }
                }
            }

            int[] starts = new int[StateCount + 1];
            int total = 0;
            for (int i = 0; i < StateCount; i++)
            {
                starts[i] = total;
                total += counts[i + 1];
            }
            starts[StateCount] = total;

            int[] cursor = new int[StateCount];
            Array.Copy(starts, cursor, StateCount);
            int[] sources = new int[total];
            for (int state = 0; state < StateCount; state++)
            {
                Decode(state, out int qy, out int qv, out int dj);
                for (int input = 0; input < 4; input++)
                {
                    if (Step(landable, grounded, qy, qv, dj, input, out int ny, out int nv, out int ndj))
                    {
                        sources[cursor[Encode(ny, nv, ndj)]++] = state;
                    }
                }
            }

            ushort[] distance = new ushort[StateCount];
            Array.Fill(distance, (ushort)MaxStored);
            int[] queue = new int[StateCount];
            int head = 0, tail = 0;
            int goalY = Math.Clamp(goalRow * 10, 0, QH - 1);
            for (int qv = MinQV; qv <= MaxQV; qv++)
            {
                for (int dj = 0; dj < 2; dj++)
                {
                    int state = Encode(goalY, qv, dj);
                    if (distance[state] == 0) continue;
                    distance[state] = 0;
                    queue[tail++] = state;
                }
            }

            while (head < tail)
            {
                int state = queue[head++];
                int next = distance[state] + 1;
                if (next >= MaxStored) continue;
                for (int e = starts[state]; e < starts[state + 1]; e++)
                {
                    int source = sources[e];
                    if (distance[source] <= next) continue;
                    distance[source] = (ushort)next;
                    queue[tail++] = source;
                }
            }

            return distance;
        }

        /// <summary>
        /// Lower bound for a fractional state. Every neighbouring cell of the quantised lattice is
        /// itself reachable, so the smallest of them is still a lower bound.
        /// </summary>
        public static uint At(ushort[] table, double y, double v, bool canDJump)
        {
            int yLo = Math.Clamp((int)Math.Floor(y * 10), 0, QH - 1);
            int yHi = Math.Clamp((int)Math.Ceiling(y * 10), 0, QH - 1);
            int vLo = Math.Clamp((int)Math.Floor(v * 10), MinQV, MaxQV);
            int vHi = Math.Clamp((int)Math.Ceiling(v * 10), MinQV, MaxQV);
            int dj = canDJump ? 1 : 0;
            uint best = MaxStored;
            best = Math.Min(best, table[Encode(yLo, vLo, dj)]);
            best = Math.Min(best, table[Encode(yHi, vLo, dj)]);
            best = Math.Min(best, table[Encode(yLo, vHi, dj)]);
            best = Math.Min(best, table[Encode(yHi, vHi, dj)]);
            return best;
        }

        /// <summary>
        /// The jump test in the real engine is a hitbox test roughly ten pixels below the centre, so
        /// a row counts as ground when anything is occupied within that reach. Being generous here
        /// is what keeps the bound a relaxation.
        /// </summary>
        private static bool[] BuildGroundedRows(bool[] landable)
        {
            bool[] grounded = new bool[Map.HEIGHT];
            for (int y = 0; y < Map.HEIGHT; y++)
            {
                for (int probe = y; probe <= Math.Min(Map.HEIGHT - 1, y + 11); probe++)
                {
                    if (!landable[probe]) continue;
                    grounded[y] = true;
                    break;
                }
            }
            return grounded;
        }

        private static bool[] BuildLandableRows(CollisionMap map)
        {
            bool[] landable = new bool[Map.HEIGHT];
            for (int y = 0; y < Map.HEIGHT; y++)
            {
                for (int x = 0; x < Map.WIDTH; x++)
                {
                    if (map.Collision[x, y] != CollisionType.None)
                    {
                        landable[y] = true;
                        break;
                    }
                }
            }
            return landable;
        }

        private static int Encode(int qy, int qv, int dj) =>
            ((dj * QVCount) + (Math.Clamp(qv, MinQV, MaxQV) - MinQV)) * QH + Math.Clamp(qy, 0, QH - 1);

        private static void Decode(int state, out int qy, out int qv, out int dj)
        {
            qy = state % QH;
            int rest = state / QH;
            qv = (rest % QVCount) + MinQV;
            dj = rest / QVCount;
        }

        /// <summary>
        /// One abstract frame. Mirrors the branch structure of <see cref="Player.Update"/> but with
        /// every geometry test replaced by the row level "landable" test.
        /// </summary>
        private static bool Step(bool[] landable, bool[] grounded, int qy, int qv, int dj, int input, out int nqy, out int nqv, out int ndj)
        {
            nqy = 0;
            nqv = 0;
            ndj = 0;

            double y = qy / 10.0;
            double v = Math.Clamp(qv / 10.0, -MaxFall, MaxFall);
            bool jump = (input & 1) != 0;
            bool release = (input & 2) != 0;

            if (jump)
            {
                if (IsGrounded(grounded, y) || dj != 0)
                {
                    v = -SingleJump;
                    dj = 1;
                }
                else if (dj != 0)
                {
                    v = -DoubleJump;
                    dj = 0;
                }
            }

            if (release && v < 0)
            {
                v *= PhysicsParams.RELEASE_MULTIPLIER;
            }

            v += Gravity;
            y += v;
            if (y < 0 || y > Map.HEIGHT - 1) return false;

            if (IsLandable(landable, y))
            {
                v = 0;
                dj = 1;
            }

            nqy = Math.Clamp((int)Math.Round(y * 10), 0, QH - 1);
            nqv = Math.Clamp((int)Math.Round(v * 10), MinQV, MaxQV);
            ndj = dj;
            return true;
        }

        private static bool IsGrounded(bool[] grounded, double y)
        {
            int row = (int)Math.Round(y);
            return (uint)row < Map.HEIGHT && grounded[row];
        }

        private static bool IsLandable(bool[] landable, double y)
        {
            int row = (int)Math.Round(y);
            return (uint)row < Map.HEIGHT && landable[row];
        }
    }
}
