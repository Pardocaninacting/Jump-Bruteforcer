# Jump-Bruteforcer
A fun tool for automatically TASing Needle screens. It searches for the fastest input sequence to get from a user-chosen starting point to a user-chosen goal point
and outputs the complete set of inputs, as well as visualizations of the player's trajectory and the parts of the map that were explored. It can also export a macro that can be used with 
[this fork of gm8emulator with macro support](https://github.com/TheBiob/OpenGMK) to create a TAS.
## Usage

https://github.com/namelessiw/Jump-Bruteforcer/assets/33900090/5dec5ee6-e012-4caa-915c-b99553e772f1

This is an example of using the bruteforcer on [Decession](https://delicious-fruit.com/ratings/game_details.php?id=19802). The map is not an exact reproduction of the normal game visuals. 
Instead, it is a transformation of the original visuals, the way it would look to a player shrunk from the normal size of 11X21 pixels down to 1 pixel.
Click 'Import Map' and select a jmap, cmap, or gm82 instances.txt file to start. You can left click to set the start and right click to set the goal.
Or you can type the coordinates into the textboxes.
Then you can click 'Start search' and get up and grab a cup of coffee. Hopefully it will be done when you get back.
You can right click the inputs list to copy it to the clipboard.
Tick 'Disable cactus' before starting a search to forbid strats that release the jump key more than once per press.
Tick 'Optimal mode' to guarantee the shortest input sequence: the search first finds a solution as fast as it can, then sweeps the state space layer by layer inside that bound. It is roughly twice as slow as the default search, which is very fast but can be a couple of frames off on large screens.

The bruteforcer supports solids, playerkillers, water (1, 2, & 3), platforms, vines, and gravity flippers.
Notable unsupported objects include anything dynamic (jump refreshers, apple animations, etc.).

## Performance
Measured on the maps in `TestBrute/jmaps`, against the state of the search before the optimisation pass:

| screen | before | after | states visited |
|---|---|---|---|
| `i_wanna_x` (1103 frames) | 49.4 s | **12.7 s** | 166.0M → 85.0M |
| `nameless` (575 frames) | 28.2 s | **6.8 s** | 94.1M → 47.6M |
| `decession` (230 frames) | 28.5 s | **17.5 s** | 87.8M → 87.8M (has vines) |

The full 67 screen regression suite went from 47 minutes to 14. `TestBrute/TestBench.cs` reproduces these numbers:
`dotnet test TestBrute\TestBrute.csproj -c Release --filter FullyQualifiedName~TestBench --logger "console;verbosity=detailed"`.
The gains come from three changes that do not alter which states the search visits: dropping `FacingRight` from the deduplication key on maps without vines (it is unobservable there), replacing the binary heap with an integer bucket queue, and sharing one `Player.Update` between the three horizontal inputs of a frame.

## Limitations
- Memory and time intensive for large-screens
- The strats will make liberal and sometimes gratuitous use of window-trick, cactusing, and one-frame stutters, and no attempt is made to make them human-friendly
- It is precise up to a 0.2 range of v-aligns, but cannot guarantee finding strats for jumps more precise than that
- The targeted fangame engine is Yuuutu.
- The mapsize is fixed at 801X609.
- no A/D support
- The default search closes a state when it is first discovered and deduplicates on a quantised key, so it is not guaranteed to return the shortest path; use 'Optimal mode' when that matters

## How it works
An A* search is performed over the state transition graph, starting from the start node and ending at the goal node, using the distance from the goal as a heuristic.
This distance weights vertical movement less than horizontal movement because jumping/falling is faster than walking.
'Optimal mode' reuses that solution as a frame bound and then runs a layered breadth first sweep, which is exhaustive inside the bound and therefore returns the true optimum.

