![RocketSim logo](https://media.discordapp.net/attachments/1433039739996344424/1495335804711735416/rocketsim.png?ex=69e5df7d&is=69e48dfd&hm=65a46e4438a241d07f97c257ad0e5ef41ae0019e9e13d5f12d5c5a09cf7dc9d6&=&format=webp&quality=lossless&width=1765&height=524)

**A Rust library for simulating Rocket League games at maximum efficiency**

This is an optimized port of RocketSim to Rust, by ZealanL and VirxEC.

## What is this?

This branch is an attempt and simplifying and optimizing RocketSim.<br>
It is based off of Virx's rebuild of Bullet in Rust.<br>

> [!CAUTION]
> This project is currently a massive WIP, and highly experimental.<br>
> Many features found in RocketSim are currently missing.

## How accurate is it?

This version of RocketSim is **not** as accurate in its current state as the version on the *main* branch.<br>
To see a list of use cases that RocketSim is suitable for, see that branch.

## What's done?

Only the **Soccar** gamemode is close to completion, with no other gamemodes at the moment.<br>
There is a built-in visualizer implemented for viewing the simulation as it runs, unlike the C++ versions which require you to get a visualizer yourself.

## OK, but how *fast* is it?

Right now, it roughly mirrors the speed of the *main* branch, although this can change and fluctuate depending on car count and other parameters.

## How do I use it?

Right now, this version of RocketSim is **not** for typical usage. Things are changing all the time, and almost nothing is actually done.<br>
I recommend you just stick to Virx's Rust bindings (https://github.com/VirxEC/rocketsim-rs) for now if you really want to use Rust.<br><br>

> [!IMPORTANT]
> **This is a legal notice directed at Epic Games / Psyonix.**<BR>
> RocketSim was written to replicate Rocket League's game logic, but does not actually contain any code from the game.<br>
> If there is an issue with this, let me (@zealanl) know on Discord and we can resolve it.
