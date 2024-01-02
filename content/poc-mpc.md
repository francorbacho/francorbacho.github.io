---
title: 'Proof of Concept MPC'
date: 2023-10-27T11:15:05+02:00
draft: false
---

Hello there! This is my attempt at creating a basic MPC. It's not perfect, and
sometimes it may get stuck, but the fundamental concept is there. Each frame, it
predicts what set of inputs minimize the cost function.

These inputs consist of a set of randomly generated pairs, as well as some basic
pairs that are commonly good (accelerate w/o steering, accelerate left, accelerate
right, etc...).

{{< poc-mpc >}}
