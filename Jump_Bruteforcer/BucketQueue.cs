namespace Jump_Bruteforcer
{
    /// <summary>
    /// Integer bucket queue used in place of a binary heap.
    ///
    /// The A* priority is a small non negative integer (a frame count), and ties have to pop
    /// last-in-first-out, which a stack per bucket gives for free. Push and pop are O(1) with no
    /// comparisons and no element shuffling, and popped entries are recycled through a free list so
    /// the backing storage is bounded by the peak frontier instead of by the number of states ever
    /// discovered.
    ///
    /// States whose heuristic is unreachable ("inside geometry") cannot be given a finite priority;
    /// they are parked on a separate stack that is only drained once every bucket is empty.
    /// </summary>
    internal sealed class BucketQueue
    {
        private struct Entry
        {
            public SearchNode Node;
            public uint Cost;
            public int Next;
        }

        private Entry[] _entries = new Entry[1024];
        private int[] _heads = new int[4096];
        private int _freeHead = -1;
        private int _farHead = -1;
        private int _count;
        private int _live;
        private int _minBucket;

        public BucketQueue()
        {
            Array.Fill(_heads, -1);
        }

        public int Count => _live;

        public void Push(SearchNode node, uint cost, uint heuristic, int weight)
        {
            int index = _freeHead >= 0 ? _freeHead : _entries.Length > _count ? _count : Grow();
            if (_freeHead >= 0)
            {
                _freeHead = _entries[index].Next;
            }
            else
            {
                _count++;
            }

            _live++;
            if (heuristic == uint.MaxValue)
            {
                _entries[index] = new Entry { Node = node, Cost = cost, Next = _farHead };
                _farHead = index;
                return;
            }

            int bucket = BucketOf(cost, SearchNode.ScaledDistance(heuristic, weight));
            _entries[index] = new Entry { Node = node, Cost = cost, Next = _heads[bucket] };
            _heads[bucket] = index;
            if (bucket < _minBucket) _minBucket = bucket;
        }

        public bool TryPop(out SearchNode node, out uint cost)
        {
            while (_minBucket < _heads.Length && _heads[_minBucket] < 0) _minBucket++;
            int index;
            if (_minBucket < _heads.Length)
            {
                index = _heads[_minBucket];
                _heads[_minBucket] = _entries[index].Next;
            }
            else if (_farHead >= 0)
            {
                index = _farHead;
                _farHead = _entries[index].Next;
            }
            else
            {
                node = default;
                cost = 0;
                return false;
            }

            node = _entries[index].Node;
            cost = _entries[index].Cost;
            _entries[index].Next = _freeHead;
            _freeHead = index;
            _live--;
            return true;
        }

        /// <summary>The states that are still waiting, used for the explored state heat map.</summary>
        public IEnumerable<SearchNode> LiveNodes
        {
            get
            {
                for (int bucket = 0; bucket < _heads.Length; bucket++)
                {
                    for (int i = _heads[bucket]; i >= 0; i = _entries[i].Next)
                    {
                        yield return _entries[i].Node;
                    }
                }
                for (int i = _farHead; i >= 0; i = _entries[i].Next)
                {
                    yield return _entries[i].Node;
                }
            }
        }

        private int BucketOf(uint cost, uint scaledHeuristic)
        {
            ulong f = (ulong)cost + scaledHeuristic;
            if (f >= (ulong)_heads.Length) GrowHeads((int)Math.Min(f + 1, int.MaxValue - 1));
            return (int)f;
        }

        private int Grow()
        {
            int index = _entries.Length;
            Array.Resize(ref _entries, _entries.Length * 2);
            return index;
        }

        private void GrowHeads(int needed)
        {
            int size = _heads.Length;
            while (size < needed) size = size > int.MaxValue / 2 ? int.MaxValue : size * 2;
            int previous = _heads.Length;
            Array.Resize(ref _heads, size);
            Array.Fill(_heads, -1, previous, size - previous);
        }
    }
}
