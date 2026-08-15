import * as React from "react"

import { cn } from "@/lib/utils"

export function Badge({ className, ...props }: React.HTMLAttributes<HTMLDivElement>) {
  return (
    <div
      className={cn(
        "inline-flex h-6 items-center gap-1.5 rounded-full border border-zinc-200 bg-white px-2.5 font-mono text-[10px] font-medium uppercase tracking-[0.12em] text-zinc-700",
        className,
      )}
      {...props}
    />
  )
}
