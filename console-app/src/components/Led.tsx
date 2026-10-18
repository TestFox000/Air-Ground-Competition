import { cn } from "@/lib/utils";

type Tone = "hud" | "amber" | "red" | "idle";

const TONE_CLASS: Record<Tone, string> = {
  hud: "bg-hud animate-pulse-dot",
  amber: "bg-amber-400 animate-pulse-dot-amber",
  red: "bg-red-400 animate-pulse-dot-red",
  idle: "bg-muted-foreground/50",
};

/** 状态 LED 点 */
export function Led({ tone = "idle", className }: { tone?: Tone; className?: string }) {
  return <span className={cn("inline-block h-2 w-2 shrink-0 rounded-full", TONE_CLASS[tone], className)} />;
}
