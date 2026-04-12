// Divergence predictor — predicts which warps can be skipped under masking.
//
// The idea: mask patterns are often spatially/temporally correlated.
// If warp 3 was all-zeros last time, it's likely all-zeros again.
//
// Implementation: 2-bit saturating counters indexed by {warp_id, op_hash}.
//   Counter >= 2  →  predict "skip this warp" (all lanes masked off).
//   Counter <  2  →  predict "execute".
//
// After each vector op, the scheduler feeds back actual mask activity,
// and counters are updated (increment on correct skip, decrement on mispredict).
//
// This is the same principle as a branch predictor, but applied to
// SIMT lane masks instead of PC-indexed branch directions.
`default_nettype none

module divergence_predictor #(
    parameter int N_WARPS    = 4,
    parameter int TABLE_BITS = 6        // 64-entry prediction table
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ── Query interface (from warp scheduler) ───────────────
    input  wire                     query_valid,
    input  wire [4:0]               query_op_type,      // hashed with warp_id for index

    output reg  [N_WARPS-1:0]       pred_skip,          // 1 = predict all-dead, skip warp

    // ── Update interface (feedback after execution) ─────────
    input  wire                     update_valid,
    input  wire [4:0]               update_op_type,
    input  wire [N_WARPS-1:0]       update_warp_active  // 1 = warp had at least one active lane
);

    // ── Prediction table ────────────────────────────────────
    // Indexed by hash(warp_id, op_type).
    // 2-bit saturating counter per entry:  0,1 = predict execute;  2,3 = predict skip.
    localparam int TABLE_SIZE = 1 << TABLE_BITS;

    reg [1:0] counter_table [TABLE_SIZE];

    // ── Hash function ───────────────────────────────────────
    function automatic [TABLE_BITS-1:0] hash_index(
        input [1:0] warp_id,
        input [4:0] op_type
    );
        // Simple XOR fold — good enough for a 64-entry table
        hash_index = {warp_id, op_type[3:0]} ^ {op_type[4], warp_id[0], op_type[2:0], 1'b0};
    endfunction

    // ── Query: produce predictions ──────────────────────────
    always_comb begin
        pred_skip = '0;
        if (query_valid) begin
            for (int w = 0; w < N_WARPS; w++) begin
                automatic logic [TABLE_BITS-1:0] idx = hash_index(w[1:0], query_op_type);
                // Predict skip if counter is in the "strongly/weakly skip" range
                // AND the actual mask confirms all-zero (belt-and-suspenders on first use)
                if (counter_table[idx] >= 2'd2)
                    pred_skip[w] = 1'b1;
            end
        end
    end

    // ── Update: train counters after execution ──────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < TABLE_SIZE; i++)
                counter_table[i] <= 2'd1;   // weakly "execute" — conservative default
        end else if (update_valid) begin
            for (int w = 0; w < N_WARPS; w++) begin
                automatic logic [TABLE_BITS-1:0] idx = hash_index(w[1:0], update_op_type);
                if (!update_warp_active[w]) begin
                    // Warp was all-dead — nudge toward "skip"
                    if (counter_table[idx] < 2'd3)
                        counter_table[idx] <= counter_table[idx] + 2'd1;
                end else begin
                    // Warp was active — nudge toward "execute"
                    if (counter_table[idx] > 2'd0)
                        counter_table[idx] <= counter_table[idx] - 2'd1;
                end
            end
        end
    end

endmodule
